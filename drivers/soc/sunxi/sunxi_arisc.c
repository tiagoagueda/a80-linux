// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the AR100 (ARISC) power-management coprocessor on Allwinner A80.
 *
 * The A80 cannot power up its Cortex-A15 cluster from the ARM side alone: the
 * vendor mcpm code delegates that to this coprocessor and refuses to try
 * unless it is running. This driver loads the AR100 firmware, completes its
 * startup handshake, and exposes the DVFS request that powers the cluster.
 *
 * The startup handshake is the part nobody documents. The firmware sends a
 * type-0x90 message HARDSYN and then spins waiting for the ARM to hand it
 * back - and that spin happens before it enables its own interrupts. Miss it
 * and the coprocessor is alive, fully initialised, and deaf forever.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/soc/sunxi/arisc.h>

#define ARISC_SRAM_SIZE		0x28000
#define ARISC_POOL_START	0x27000		/* message pool, SRAM-relative */
#define ARISC_MSG_SLOT		(ARISC_POOL_START + 0x40)
#define ARISC_MSG_LEN		64

/*
 * v0.0.80 keeps the per-cluster CPU voltage identity in a BSS global:
 *   0x10a50 + 8  = cluster 0 (CPU-L)
 *   0x10a50 + 12 = cluster 1 (CPU-B)
 * The DVFS handler passes it straight to its set/get voltage helpers.
 * It is zero unless the firmware's own dvfs init filled it in, and type
 * 0 is AXP809 DCDC1 - a chip this board does not have - so every CPU-B
 * voltage change fails and 0x30 returns -1.
 */
#define ARISC_DVFS_STATE	0x10a50
#define ARISC_DVFS_CPUB_ID	(ARISC_DVFS_STATE + 12)
#define ARISC_VOLT_TYPE_OZ80120	29

/* Message header, as the firmware lays it out. paras[] starts at byte 20. */
#define MSG_STATE		0x00
#define MSG_ATTR		0x01
#define MSG_TYPE		0x02
#define MSG_RESULT		0x03
#define MSG_PARAS		0x14

#define MSG_STATE_INITIALIZED	2
#define MSG_STATE_PROCESSED	5
#define MSG_ATTR_HARDSYN	2

#define MSG_TYPE_DVFS_REQ	0x30
#define MSG_TYPE_DVFS_CFG_VF	0x31
#define MSG_TYPE_SET_PMU_VOLT	0x55
#define MSG_TYPE_GET_PMU_VOLT	0x56
#define MSG_TYPE_LOOPBACK	0x61
#define MSG_TYPE_STARTUP	0x90

/* hwmsgbox: ARM receives on 0/2/5, transmits on 4, feeds startup back on 3 */
#define MBOX_FIFO_STAT(n)	(0x0100 + 4 * (n))
#define MBOX_MSG_STAT(n)	(0x0140 + 4 * (n))
#define MBOX_MSG_DATA(n)	(0x0180 + 4 * (n))
#define ARISC_TX_CH		4
#define ARISC_FEEDBACK_CH	3

#define ARISC_BEACON_TIMEOUT_MS	2000
#define ARISC_XACT_TIMEOUT_MS	3000

static const int arisc_rx_ch[] = { 0, 2, 5 };

struct sunxi_arisc {
	struct device	*dev;
	void __iomem	*sram;
	void __iomem	*mbox;
	void __iomem	*prcm;
	struct mutex	lock;		/* serialises the single message slot */
	struct delayed_work fw_work;	/* firmware arrives after the rootfs */
	int		fw_tries;
	bool		ready;
};

static struct sunxi_arisc *arisc;

static void arisc_msg_set(struct sunxi_arisc *ar, u32 off, u8 type)
{
	memset_io(ar->sram + off, 0, ARISC_MSG_LEN);
	writeb(MSG_STATE_INITIALIZED, ar->sram + off + MSG_STATE);
	writeb(MSG_ATTR_HARDSYN, ar->sram + off + MSG_ATTR);
	writeb(type, ar->sram + off + MSG_TYPE);
}

static void arisc_msg_para(struct sunxi_arisc *ar, u32 off, int i, u32 v)
{
	writel(v, ar->sram + off + MSG_PARAS + 4 * i);
}

static u32 arisc_msg_para_get(struct sunxi_arisc *ar, u32 off, int i)
{
	return readl(ar->sram + off + MSG_PARAS + 4 * i);
}

/*
 * Send one message and wait for the coprocessor to hand the same handle back.
 * Offsets on the wire are SRAM-relative in both directions.
 */
static int arisc_send(struct sunxi_arisc *ar, u32 off)
{
	int i, t;

	for (t = 0; t < 500; t++) {
		if (readl(ar->mbox + MBOX_FIFO_STAT(ARISC_TX_CH)) != 1)
			break;
		udelay(100);
	}
	writel(off, ar->mbox + MBOX_MSG_DATA(ARISC_TX_CH));

	for (t = 0; t < ARISC_XACT_TIMEOUT_MS * 10; t++) {
		for (i = 0; i < ARRAY_SIZE(arisc_rx_ch); i++) {
			int ch = arisc_rx_ch[i];

			if (!readl(ar->mbox + MBOX_MSG_STAT(ch)))
				continue;
			if (readl(ar->mbox + MBOX_MSG_DATA(ch)) == off)
				return readb(ar->sram + off + MSG_RESULT);
		}
		udelay(100);
	}
	return -ETIMEDOUT;
}

/* Wait for the startup beacon and return its handle, so we can hand it back. */
static int arisc_wait_beacon(struct sunxi_arisc *ar)
{
	int i, t;

	for (t = 0; t < ARISC_BEACON_TIMEOUT_MS; t++) {
		for (i = 0; i < ARRAY_SIZE(arisc_rx_ch); i++) {
			int ch = arisc_rx_ch[i];
			u32 handle;

			if (!readl(ar->mbox + MBOX_MSG_STAT(ch)))
				continue;

			handle = readl(ar->mbox + MBOX_MSG_DATA(ch));
			if (handle >= ARISC_SRAM_SIZE - ARISC_MSG_LEN)
				continue;
			if (readb(ar->sram + handle + MSG_TYPE) != MSG_TYPE_STARTUP)
				continue;

			dev_info(ar->dev, "startup beacon on ch%d (handle 0x%05x)\n",
				 ch, handle);
			return handle;
		}
		mdelay(1);
	}
	return -ETIMEDOUT;
}

int sunxi_arisc_dvfs_set_cpufreq(u32 freq_khz, u32 pll)
{
	int ret;

	if (!arisc || !arisc->ready)
		return -EAGAIN;
	if (pll != 1 && pll != 2)
		return -EINVAL;

	mutex_lock(&arisc->lock);
	arisc_msg_set(arisc, ARISC_MSG_SLOT, MSG_TYPE_DVFS_REQ);
	arisc_msg_para(arisc, ARISC_MSG_SLOT, 0, freq_khz);	/* KHz, not Hz */
	arisc_msg_para(arisc, ARISC_MSG_SLOT, 1, pll);		/* 2 = A15 */
	ret = arisc_send(arisc, ARISC_MSG_SLOT);
	mutex_unlock(&arisc->lock);

	if (ret)
		dev_err(arisc->dev, "dvfs pll%u %u kHz failed: %d\n",
			pll, freq_khz, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(sunxi_arisc_dvfs_set_cpufreq);

int sunxi_arisc_dvfs_cfg_vf_table(unsigned int cluster,
				  const struct sunxi_arisc_vf *vf, int n)
{
	int i, ret = 0;

	if (!arisc || !arisc->ready)
		return -EAGAIN;

	mutex_lock(&arisc->lock);
	for (i = 0; i < 16; i++) {
		arisc_msg_set(arisc, ARISC_MSG_SLOT, MSG_TYPE_DVFS_CFG_VF);
		arisc_msg_para(arisc, ARISC_MSG_SLOT, 0, i);
		arisc_msg_para(arisc, ARISC_MSG_SLOT, 1, i < n ? vf[i].freq_hz : 0);
		arisc_msg_para(arisc, ARISC_MSG_SLOT, 2, i < n ? vf[i].volt_mv : 900);
		arisc_msg_para(arisc, ARISC_MSG_SLOT, 3, cluster ? 4 : 3);
		arisc_msg_para(arisc, ARISC_MSG_SLOT, 4, cluster);
		ret = arisc_send(arisc, ARISC_MSG_SLOT);
		if (ret) {
			dev_err(arisc->dev, "v-f table idx %d failed: %d\n", i, ret);
			break;
		}
	}
	mutex_unlock(&arisc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sunxi_arisc_dvfs_cfg_vf_table);

/* Firmware voltage rails; 29 is the external CPU-B DC-DC. */
int sunxi_arisc_get_volt(u32 type)
{
	int ret;

	if (!arisc || !arisc->ready)
		return -EAGAIN;
	mutex_lock(&arisc->lock);
	arisc_msg_set(arisc, ARISC_MSG_SLOT, MSG_TYPE_GET_PMU_VOLT);
	arisc_msg_para(arisc, ARISC_MSG_SLOT, 0, type);
	ret = arisc_send(arisc, ARISC_MSG_SLOT);
	if (!ret)
		ret = arisc_msg_para_get(arisc, ARISC_MSG_SLOT, 1);
	mutex_unlock(&arisc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sunxi_arisc_get_volt);

int sunxi_arisc_set_volt(u32 type, u32 mv)
{
	int ret;

	if (!arisc || !arisc->ready)
		return -EAGAIN;
	mutex_lock(&arisc->lock);
	arisc_msg_set(arisc, ARISC_MSG_SLOT, MSG_TYPE_SET_PMU_VOLT);
	arisc_msg_para(arisc, ARISC_MSG_SLOT, 0, type);
	arisc_msg_para(arisc, ARISC_MSG_SLOT, 1, mv);
	ret = arisc_send(arisc, ARISC_MSG_SLOT);
	mutex_unlock(&arisc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sunxi_arisc_set_volt);

bool sunxi_arisc_is_ready(void)
{
	return arisc && arisc->ready;
}
EXPORT_SYMBOL_GPL(sunxi_arisc_is_ready);

static int sunxi_arisc_start(struct sunxi_arisc *ar, const struct firmware *fw)
{
	int beacon, ret, i;

	if (fw->size > ARISC_POOL_START) {
		dev_err(ar->dev, "firmware too large: %zu\n", fw->size);
		return -EINVAL;
	}

	/* hold in reset, load, release */
	writel(readl(ar->prcm) & ~1u, ar->prcm);
	usleep_range(50000, 60000);

	memset_io(ar->sram, 0, ARISC_SRAM_SIZE);
	memcpy_toio(ar->sram, fw->data, fw->size);

	/* drain what the ARM can pop; TX channels do not clear this way */
	for (i = 0; i < ARRAY_SIZE(arisc_rx_ch); i++) {
		int guard = 16;

		while (readl(ar->mbox + MBOX_MSG_STAT(arisc_rx_ch[i])) && guard--)
			readl(ar->mbox + MBOX_MSG_DATA(arisc_rx_ch[i]));
	}

	writel(readl(ar->prcm) | 1u, ar->prcm);

	beacon = arisc_wait_beacon(ar);
	if (beacon < 0) {
		dev_err(ar->dev, "no startup beacon; coprocessor did not start\n");
		return beacon;
	}

	/*
	 * Hand the beacon back. The firmware compares the echoed handle against
	 * the message address and only then runs cpsie i. Without this it never
	 * enables interrupts and ignores everything we send.
	 */
	writeb(MSG_STATE_PROCESSED, ar->sram + beacon + MSG_STATE);
	writeb(0, ar->sram + beacon + MSG_RESULT);
	writel(beacon, ar->mbox + MBOX_MSG_DATA(ARISC_FEEDBACK_CH));

	/* prove the round trip before declaring ourselves up */
	mutex_lock(&ar->lock);
	arisc_msg_set(ar, ARISC_MSG_SLOT, MSG_TYPE_LOOPBACK);
	arisc_msg_para(ar, ARISC_MSG_SLOT, 0, 0xdeadbeef);
	ret = arisc_send(ar, ARISC_MSG_SLOT);
	if (!ret && arisc_msg_para_get(ar, ARISC_MSG_SLOT, 0) != 0xdeadbeef)
		ret = -EIO;
	mutex_unlock(&ar->lock);

	if (ret) {
		dev_err(ar->dev, "loopback failed: %d\n", ret);
		return ret < 0 ? ret : -EIO;
	}

	/* publish early so the volt helpers below can use the link */
	arisc = ar;
	ar->ready = true;
	/* point the CPU-B voltage helpers at the external DC-DC */
	dev_info(ar->dev, "CPUB voltage id was %u, setting %u\n",
		 readl(ar->sram + ARISC_DVFS_CPUB_ID),
		 ARISC_VOLT_TYPE_OZ80120);
	writel(ARISC_VOLT_TYPE_OZ80120, ar->sram + ARISC_DVFS_CPUB_ID);

	dev_info(ar->dev,
		 "coprocessor up, loopback ok; CPUB rail reads %d mV\n",
		 sunxi_arisc_get_volt(29));
	return 0;
}

/*
 * The coprocessor is not needed until userspace asks for a CPU or a
 * frequency, and its firmware lives on the rootfs, which is not mounted
 * when we probe. Retry in the background rather than failing the probe.
 */
static void sunxi_arisc_fw_work(struct work_struct *work)
{
	struct sunxi_arisc *ar = container_of(to_delayed_work(work),
					      struct sunxi_arisc, fw_work);
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, "sun9i-a80-arisc.bin", ar->dev);
	if (ret) {
		if (++ar->fw_tries < 30) {
			schedule_delayed_work(&ar->fw_work,
					      msecs_to_jiffies(2000));
			return;
		}
		dev_err(ar->dev, "giving up on sun9i-a80-arisc.bin: %d\n", ret);
		return;
	}

	ret = sunxi_arisc_start(ar, fw);
	release_firmware(fw);
	if (ret)
		return;

	ar->ready = true;
	arisc = ar;
}

static int sunxi_arisc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_arisc *ar;

	ar = devm_kzalloc(dev, sizeof(*ar), GFP_KERNEL);
	if (!ar)
		return -ENOMEM;
	ar->dev = dev;
	mutex_init(&ar->lock);
	INIT_DELAYED_WORK(&ar->fw_work, sunxi_arisc_fw_work);

	ar->sram = of_iomap(dev->of_node, 0);
	ar->mbox = of_iomap(dev->of_node, 1);
	ar->prcm = of_iomap(dev->of_node, 2);
	if (!ar->sram || !ar->mbox || !ar->prcm) {
		dev_err(dev, "failed to map sram/msgbox/prcm\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ar);
	schedule_delayed_work(&ar->fw_work, msecs_to_jiffies(1000));
	return 0;
}

static const struct of_device_id sunxi_arisc_of_match[] = {
	{ .compatible = "allwinner,sun9i-a80-arisc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_arisc_of_match);

static struct platform_driver sunxi_arisc_driver = {
	.probe	= sunxi_arisc_probe,
	.driver	= {
		.name		= "sunxi-arisc",
		.of_match_table	= sunxi_arisc_of_match,
	},
};
module_platform_driver(sunxi_arisc_driver);

MODULE_DESCRIPTION("Allwinner A80 AR100 coprocessor driver");
MODULE_LICENSE("GPL");
