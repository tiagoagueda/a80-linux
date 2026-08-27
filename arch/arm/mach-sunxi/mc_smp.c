// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 Chen-Yu Tsai
 *
 * Chen-Yu Tsai <wens@csie.org>
 *
 * arch/arm/mach-sunxi/mc_smp.c
 *
 * Based on Allwinner code, arch/arm/mach-exynos/mcpm-exynos.c, and
 * arch/arm/mach-hisi/platmcpm.c
 * Cluster cache enable trampoline code adapted from MCPM framework
 */

#include <linux/arm-cci.h>
#include <linux/cpu_pm.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/soc/sunxi/arisc.h>
#include <linux/iopoll.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/cp15.h>
#include <asm/cputype.h>
#include <asm/idmap.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

#define SUNXI_CPUS_PER_CLUSTER		4
#define SUNXI_NR_CLUSTERS		2

#define POLL_USEC	100
#define TIMEOUT_USEC	100000

#define CPUCFG_CX_CTRL_REG0(c)		(0x10 * (c))
#define CPUCFG_CX_CTRL_REG0_L1_RST_DISABLE(n)	BIT(n)
#define CPUCFG_CX_CTRL_REG0_L1_RST_DISABLE_ALL	0xf
#define CPUCFG_CX_CTRL_REG0_L2_RST_DISABLE_A7	BIT(4)
#define CPUCFG_CX_CTRL_REG0_L2_RST_DISABLE_A15	BIT(0)
#define CPUCFG_CX_CTRL_REG1(c)		(0x10 * (c) + 0x4)
#define CPUCFG_CX_CTRL_REG1_ACINACTM	BIT(0)
#define CPUCFG_CX_STATUS(c)		(0x30 + 0x4 * (c))
#define CPUCFG_CX_STATUS_STANDBYWFI(n)	BIT(16 + (n))
#define CPUCFG_CX_STATUS_STANDBYWFIL2	BIT(0)
#define CPUCFG_CX_RST_CTRL(c)		(0x80 + 0x4 * (c))
#define CPUCFG_CX_RST_CTRL_DBG_SOC_RST	BIT(24)
#define CPUCFG_CX_RST_CTRL_ETM_RST(n)	BIT(20 + (n))
#define CPUCFG_CX_RST_CTRL_ETM_RST_ALL	(0xf << 20)
#define CPUCFG_CX_RST_CTRL_DBG_RST(n)	BIT(16 + (n))
#define CPUCFG_CX_RST_CTRL_DBG_RST_ALL	(0xf << 16)
#define CPUCFG_CX_RST_CTRL_H_RST	BIT(12)
#define CPUCFG_CX_RST_CTRL_L2_RST	BIT(8)
#define CPUCFG_CX_RST_CTRL_CX_RST(n)	BIT(4 + (n))
#define CPUCFG_CX_RST_CTRL_CX_RST_ALL	(0xf << 4)
#define CPUCFG_CX_RST_CTRL_CORE_RST(n)	BIT(n)
#define CPUCFG_CX_RST_CTRL_CORE_RST_ALL	(0xf << 0)

#define PRCM_CPU_PO_RST_CTRL(c)		(0x4 + 0x4 * (c))
#define PRCM_CPU_PO_RST_CTRL_CORE(n)	BIT(n)
#define PRCM_CPU_PO_RST_CTRL_CORE_ALL	0xf
#define PRCM_PWROFF_GATING_REG(c)	(0x100 + 0x4 * (c))
/* The power off register for clusters are different from a80 and a83t */
#define PRCM_PWROFF_GATING_REG_CLUSTER_SUN8I	BIT(0)
#define PRCM_PWROFF_GATING_REG_CLUSTER_SUN9I	BIT(4)
#define PRCM_PWROFF_GATING_REG_CORE(n)	BIT(n)
#define PRCM_PWR_SWITCH_REG(c, cpu)	(0x140 + 0x10 * (c) + 0x4 * (cpu))
#define PRCM_CPU_SOFT_ENTRY_REG		0x164

/* R_CPUCFG registers, specific to sun8i-a83t */
#define R_CPUCFG_CLUSTER_PO_RST_CTRL(c)	(0x30 + (c) * 0x4)
#define R_CPUCFG_CLUSTER_PO_RST_CTRL_CORE(n)	BIT(n)
#define R_CPUCFG_CPU_SOFT_ENTRY_REG		0x01a4

#define CPU0_SUPPORT_HOTPLUG_MAGIC0	0xFA50392F
#define CPU0_SUPPORT_HOTPLUG_MAGIC1	0x790DCA3A

static void __iomem *cpucfg_base;
static void __iomem *prcm_base;
static void __iomem *sram_b_smp_base;
static void __iomem *r_cpucfg_base;

extern void sunxi_mc_smp_secondary_startup(void);
extern void sunxi_mc_smp_resume(void);
static bool is_a83t;

static bool sunxi_core_is_cortex_a15(unsigned int core, unsigned int cluster)
{
	struct device_node *node;
	int cpu = cluster * SUNXI_CPUS_PER_CLUSTER + core;
	bool is_compatible;

	node = of_cpu_device_node_get(cpu);

	/* In case of_cpu_device_node_get fails */
	if (!node)
		node = of_get_cpu_node(cpu, NULL);

	if (!node) {
		/*
		 * There's no point in returning an error, since we
		 * would be mid way in a core or cluster power sequence.
		 */
		pr_err("%s: Couldn't get CPU cluster %u core %u device node\n",
		       __func__, cluster, core);

		return false;
	}

	is_compatible = of_device_is_compatible(node, "arm,cortex-a15");
	of_node_put(node);
	return is_compatible;
}

/*
 * Poll a power-switch register until it reaches the expected value.
 *
 * The vendor BSP spins on these reads rather than relying on a fixed delay;
 * the daisy-chained clamp takes a variable time to settle.  Bounded here so a
 * wedged clamp cannot hang the boot.
 */
static int sunxi_cpu_power_switch_wait(unsigned int cpu, unsigned int cluster,
				       u32 want)
{
	unsigned int tries = 1000;

	while (readl(prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu)) != want) {
		if (!--tries) {
			pr_err("power clamp cluster %u cpu %u stuck, want %02x\n",
			       cluster, cpu, want);
			return -ETIMEDOUT;
		}
		udelay(10);
	}

	return 0;
}

/*
 * A80 cluster 1 (Cortex-A15) uses the OPPOSITE power-clamp convention from
 * cluster 0 (Cortex-A7):
 *
 *   A7   ramps DOWN 0xe0 -> 0x80 -> 0x00, and 0x00 means POWERED ON
 *   A15  ramps UP   0x01 -> 0x07 -> 0x1f -> 0x7f -> 0xff, 0xff means ON
 *
 * The vendor BSP dispatches on cluster for exactly this reason
 * (sun9i_cpu_power_switch_set -> sun9i_ca7/ca15_power_switch_set).  Driving the
 * A7 sequence on the A15 clamp writes 0x00, which switches that cluster OFF -
 * which is why the A15 cores never came online.
 *
 * The A83T is unaffected: both of its clusters use the A7 convention.
 */
static int sunxi_cpu_power_switch_set_a15(unsigned int cpu, unsigned int cluster,
					  bool enable)
{
	u32 reg = readl(prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));

	if (enable) {
		if (reg == 0xff) {
			pr_debug("power clamp cluster %u cpu %u already open\n",
				 cluster, cpu);
			return 0;
		}

		writel(0x01, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(20);
		writel(0x07, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0x1f, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0x7f, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0xff, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(20);

		return sunxi_cpu_power_switch_wait(cpu, cluster, 0xff);
	}

	if (reg == 0x00) {
		pr_debug("power clamp cluster %u cpu %u already closed\n",
			 cluster, cpu);
		return 0;
	}

	writel(0x00, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
	udelay(30);

	return sunxi_cpu_power_switch_wait(cpu, cluster, 0x00);
}

static int sunxi_cpu_power_switch_set(unsigned int cpu, unsigned int cluster,
				      bool enable)
{
	u32 reg;

	/* A80 cluster 1 is the A15 and inverts the clamp encoding */
	if (!is_a83t && cluster == 1)
		return sunxi_cpu_power_switch_set_a15(cpu, cluster, enable);

	/* control sequence from Allwinner A80 user manual v1.2 PRCM section */
	reg = readl(prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
	if (enable) {
		if (reg == 0x00) {
			pr_debug("power clamp for cluster %u cpu %u already open\n",
				 cluster, cpu);
			return 0;
		}

		writel(0xff, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0xfe, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0xf8, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0xf0, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0x00, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
	} else {
		writel(0xff, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
	}

	return 0;
}

static void sunxi_cpu0_hotplug_support_set(bool enable)
{
	if (enable) {
		writel(CPU0_SUPPORT_HOTPLUG_MAGIC0, sram_b_smp_base);
		writel(CPU0_SUPPORT_HOTPLUG_MAGIC1, sram_b_smp_base + 0x4);
	} else {
		writel(0x0, sram_b_smp_base);
		writel(0x0, sram_b_smp_base + 0x4);
	}
}

static int sunxi_cpu_powerup(unsigned int cpu, unsigned int cluster)
{
	u32 reg;

	pr_debug("%s: cluster %u cpu %u\n", __func__, cluster, cpu);
	if (cpu >= SUNXI_CPUS_PER_CLUSTER || cluster >= SUNXI_NR_CLUSTERS)
		return -EINVAL;

	/* Set hotplug support magic flags for cpu0 */
	if (cluster == 0 && cpu == 0)
		sunxi_cpu0_hotplug_support_set(true);

	/* assert processor power-on reset */
	reg = readl(prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));
	reg &= ~PRCM_CPU_PO_RST_CTRL_CORE(cpu);
	writel(reg, prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));

	if (is_a83t) {
		/* assert cpu power-on reset */
		reg  = readl(r_cpucfg_base +
			     R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
		reg &= ~(R_CPUCFG_CLUSTER_PO_RST_CTRL_CORE(cpu));
		writel(reg, r_cpucfg_base +
		       R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
		udelay(10);
	}

	/* Cortex-A7: hold L1 reset disable signal low */
	if (!sunxi_core_is_cortex_a15(cpu, cluster)) {
		reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG0(cluster));
		reg &= ~CPUCFG_CX_CTRL_REG0_L1_RST_DISABLE(cpu);
		writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG0(cluster));
	}

	/* assert processor related resets */
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg &= ~CPUCFG_CX_RST_CTRL_DBG_RST(cpu);

	/*
	 * Allwinner code also asserts resets for NEON on A15. According
	 * to ARM manuals, asserting power-on reset is sufficient.
	 */
	if (!sunxi_core_is_cortex_a15(cpu, cluster))
		reg &= ~CPUCFG_CX_RST_CTRL_ETM_RST(cpu);

	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));

	/* open power switch */
	sunxi_cpu_power_switch_set(cpu, cluster, true);

	/* Handle A83T bit swap */
	if (is_a83t) {
		if (cpu == 0)
			cpu = 4;
	}

	/* clear processor power gate */
	reg = readl(prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	reg &= ~PRCM_PWROFF_GATING_REG_CORE(cpu);
	writel(reg, prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	udelay(20);

	/* Handle A83T bit swap */
	if (is_a83t) {
		if (cpu == 4)
			cpu = 0;
	}

	/* de-assert processor power-on reset */
	reg = readl(prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));
	reg |= PRCM_CPU_PO_RST_CTRL_CORE(cpu);
	writel(reg, prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));

	/* the vendor settles here before releasing the core reset */
	udelay(60);

	if (is_a83t) {
		reg  = readl(r_cpucfg_base +
			     R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
		reg |= R_CPUCFG_CLUSTER_PO_RST_CTRL_CORE(cpu);
		writel(reg, r_cpucfg_base +
		       R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
		udelay(10);
	}

	/* de-assert all processor resets */
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg |= CPUCFG_CX_RST_CTRL_DBG_RST(cpu);
	reg |= CPUCFG_CX_RST_CTRL_CORE_RST(cpu);
	if (!sunxi_core_is_cortex_a15(cpu, cluster))
		reg |= CPUCFG_CX_RST_CTRL_ETM_RST(cpu);
	else
		reg |= CPUCFG_CX_RST_CTRL_CX_RST(cpu); /* NEON */
	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));

	return 0;
}

/*
 * Tronsmart Draco AW80: VDD_CPU_A15 is not a PMIC rail. It comes from an
 * external DC-DC enabled by a15_pwr_en (PL02) with a 3-bit voltage select on
 * a15_vset1..3 (PL03/04/05), per the board's own script.bin.
 *
 * Codes measured on the board (PL05,PL04,PL03): 000=1200mV 001=900mV
 * 010=1000mV 100=1100mV 101=840mV.
 *
 * The rail MUST be raised while the cluster power domain is isolated. Enabling
 * it with the per-core power switches already closed (which is how U-Boot
 * leaves them, 0xff) collapses the SoC instantly - no panic, no console.
 */
#define SUN9I_R_PIO_PBASE	0x08002c00
#define SUN9I_R_PIO_PL_CFG0	0x00
#define SUN9I_R_PIO_PL_DATA	0x10
#define A15_PWR_EN		2
#define A15_VSET_SHIFT		3
#define A15_VSET_900MV		0	/* code 000 = 1200 mV per AR100 */

static void sun9i_a15_domain_isolate(void)
{
	u32 reg;
	int i;

	/*
	 * Open every per-core power switch and gate the cores and the
	 * cluster before anyone raises VDD_CPU_A15. Energising the rail
	 * into an already-connected domain collapses the SoC instantly,
	 * with nothing on the console. The AR100 owns the rail itself.
	 */
	for (i = 0; i < SUNXI_CPUS_PER_CLUSTER; i++)
		writel(0x00, prcm_base + PRCM_PWR_SWITCH_REG(1, i));
	reg = readl(prcm_base + PRCM_PWROFF_GATING_REG(1));
	writel(reg | 0x1f, prcm_base + PRCM_PWROFF_GATING_REG(1));
	udelay(20);
}

/*
 * A80 cluster power-up is not something the ARM side can do alone: the
 * voltage rail and the cluster power sequencing live behind the AR100
 * coprocessor, and the vendor code refuses to even try unless it is up.
 * A single DVFS request performs the power-on and sets the operating
 * point; there is no separate power-on message.
 *
 * TODO: these operating points are board data and should come from the
 * cpu OPP tables rather than being hardcoded here.
 */
static const struct sunxi_arisc_vf sun9i_a7_vf[] = {
	{ 1200000000, 1020 },
	{ 1104000000,  960 },
	{ 1008000000,  900 },
	{  816000000,  840 },
};
static const struct sunxi_arisc_vf sun9i_a15_vf[] = {
	{ 2016000000, 1200 },
	{ 1800000000, 1100 },
	{ 1608000000, 1000 },
	{ 1200000000,  900 },
};

#define SUN9I_A15_BOOT_KHZ	1800000
#define SUN9I_A15_BOOT_MV	1100
#define SUN9I_A15_VOLT_TYPE	29	/* external CPU-B DC-DC */

/*
 * a15_pwr_en is PL02 on this board. The AR100 drives the three vset lines
 * (PL03..PL05) when it changes CPU-B voltage, but it never touches the
 * enable, so the rail stays off and the cores never execute even after a
 * DVFS request the coprocessor reports as successful.
 */
#define SUN9I_R_PIO_PBASE	0x08002c00
#define SUN9I_R_PIO_PL_CFG0	0x00
#define SUN9I_R_PIO_PL_DATA	0x10
#define SUN9I_A15_PWR_EN	2

static void sun9i_a15_rail_enable(void)
{
	void __iomem *rp;
	u32 reg;

	rp = ioremap(SUN9I_R_PIO_PBASE, 0x400);
	if (!rp)
		return;

	/* R_PIO is clock-gated and in reset this early; ungate it ourselves. */
	writel(readl(prcm_base + 0x28) | BIT(0), prcm_base + 0x28);
	writel(readl(prcm_base + 0xb0) | BIT(0), prcm_base + 0xb0);
	udelay(10);

	reg = readl(rp + SUN9I_R_PIO_PL_CFG0);
	reg &= ~(0xfu << (SUN9I_A15_PWR_EN * 4));
	reg |= (0x1u << (SUN9I_A15_PWR_EN * 4));
	writel(reg, rp + SUN9I_R_PIO_PL_CFG0);
	writel(readl(rp + SUN9I_R_PIO_PL_DATA) | BIT(SUN9I_A15_PWR_EN),
	       rp + SUN9I_R_PIO_PL_DATA);
	mdelay(10);

	pr_info("sunxi mc_smp: a15_pwr_en asserted, PL CFG0=%08x DATA=%08x\n",
		readl(rp + SUN9I_R_PIO_PL_CFG0),
		readl(rp + SUN9I_R_PIO_PL_DATA));
	iounmap(rp);
}

static int sun9i_a15_cluster_power_on(void)
{
	int ret;

	if (!sunxi_arisc_is_ready()) {
		pr_info("sunxi mc_smp: AR100 not up yet, cannot power the A15 cluster\n");
		return -EAGAIN;
	}

	/*
	 * The vendor loads the tables for *both* clusters before issuing any
	 * DVFS request. The handler looks a voltage up by frequency, so a
	 * cluster with an empty table has nothing to raise its rail to.
	 */
	ret = sunxi_arisc_dvfs_cfg_vf_table(0, sun9i_a7_vf,
					    ARRAY_SIZE(sun9i_a7_vf));
	if (ret) {
		pr_err("sunxi mc_smp: A7 v-f table failed: %d\n", ret);
		return ret;
	}

	ret = sunxi_arisc_dvfs_cfg_vf_table(1, sun9i_a15_vf,
					    ARRAY_SIZE(sun9i_a15_vf));
	if (ret) {
		pr_err("sunxi mc_smp: A15 v-f table failed: %d\n", ret);
		return ret;
	}

	/*
	 * Put the rail at the operating point we are about to ask for. The
	 * DVFS handler sets the voltage, reads it back and fails the whole
	 * request if the two differ:
	 *   [ERR]:current voltage:%u != target voltage:%u
	 * The rail comes out of reset at 1200 mV, so asking for a lower
	 * operating point without this fails before anything is powered.
	 */
	ret = sunxi_arisc_get_volt(SUN9I_A15_VOLT_TYPE);
	pr_info("sunxi mc_smp: A15 rail at %d mV, target %d mV\n",
		ret, SUN9I_A15_BOOT_MV);
	if (ret != SUN9I_A15_BOOT_MV) {
		ret = sunxi_arisc_set_volt(SUN9I_A15_VOLT_TYPE, SUN9I_A15_BOOT_MV);
		if (ret) {
			pr_err("sunxi mc_smp: A15 rail preset failed: %d\n", ret);
			return ret;
		}
		pr_info("sunxi mc_smp: A15 rail now %d mV\n",
			sunxi_arisc_get_volt(SUN9I_A15_VOLT_TYPE));
	}
	/* pll2 selects cluster 1; freq is in kHz */
	ret = sunxi_arisc_dvfs_set_cpufreq(SUN9I_A15_BOOT_KHZ, 2);
	if (ret) {
		pr_err("sunxi mc_smp: A15 power-on failed: %d\n", ret);
		return ret;
	}

	/* voltage is set; now actually switch the rail on */
	sun9i_a15_rail_enable();
	pr_info("sunxi mc_smp: A15 cluster powered on at %u kHz\n",
		SUN9I_A15_BOOT_KHZ);
	return 0;
}

static int sunxi_cluster_powerup(unsigned int cluster)
{
	u32 reg;

	pr_debug("%s: cluster %u\n", __func__, cluster);
	if (cluster >= SUNXI_NR_CLUSTERS)
		return -EINVAL;

	/* For A83T, assert cluster cores resets */
	if (is_a83t) {
		reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
		reg &= ~CPUCFG_CX_RST_CTRL_CORE_RST_ALL;   /* Core Reset    */
		writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
		udelay(10);
	}

	/*
	 * A80 cluster 1 has no supply until its external DC-DC is enabled, and
	 * that must happen with the power domain isolated. Do it before any
	 * reset handling below.
	 */
	if (!is_a83t && cluster == 1)
		sun9i_a15_domain_isolate();

	/* assert ACINACTM */
	reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));
	reg |= CPUCFG_CX_CTRL_REG1_ACINACTM;
	writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));

	/* assert cluster processor power-on resets */
	reg = readl(prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));
	reg &= ~PRCM_CPU_PO_RST_CTRL_CORE_ALL;
	writel(reg, prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));

	/* assert cluster cores resets */
	if (is_a83t) {
		reg  = readl(r_cpucfg_base +
			     R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
		reg &= ~CPUCFG_CX_RST_CTRL_CORE_RST_ALL;
		writel(reg, r_cpucfg_base +
		       R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
		udelay(10);
	}

	/* assert cluster resets */
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg &= ~CPUCFG_CX_RST_CTRL_DBG_SOC_RST;
	reg &= ~CPUCFG_CX_RST_CTRL_DBG_RST_ALL;
	reg &= ~CPUCFG_CX_RST_CTRL_H_RST;
	reg &= ~CPUCFG_CX_RST_CTRL_L2_RST;

	/*
	 * The vendor asserts the NEON resets on A15 here and releases them
	 * with the other cluster resets below. Mainline skipped that on the
	 * grounds that power-on reset is sufficient per the ARM manuals,
	 * which holds for a cluster that already had power - but this one is
	 * coming up from a rail that was off entirely.
	 */
	if (!sunxi_core_is_cortex_a15(0, cluster))
		reg &= ~CPUCFG_CX_RST_CTRL_ETM_RST_ALL;
	else
		reg &= ~CPUCFG_CX_RST_CTRL_CX_RST_ALL;

	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));

	/* hold L1/L2 reset disable signals low */
	reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG0(cluster));
	if (sunxi_core_is_cortex_a15(0, cluster)) {
		/* Cortex-A15: hold L2RSTDISABLE low */
		reg &= ~CPUCFG_CX_CTRL_REG0_L2_RST_DISABLE_A15;
	} else {
		/* Cortex-A7: hold L1RSTDISABLE and L2RSTDISABLE low */
		reg &= ~CPUCFG_CX_CTRL_REG0_L1_RST_DISABLE_ALL;
		reg &= ~CPUCFG_CX_CTRL_REG0_L2_RST_DISABLE_A7;
	}
	writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG0(cluster));

	/*
	 * Hand off to the AR100 here, exactly where the vendor does: after
	 * L2RSTDISABLE is held low and before the cluster power gate is
	 * cleared.
	 */
	if (!is_a83t && cluster == 1) {
		int aret = sun9i_a15_cluster_power_on();

		if (aret)
			return aret;
		mdelay(1);
	}
	/* clear cluster power gate */
	reg = readl(prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	if (is_a83t)
		reg &= ~PRCM_PWROFF_GATING_REG_CLUSTER_SUN8I;
	else
		reg &= ~PRCM_PWROFF_GATING_REG_CLUSTER_SUN9I;
	writel(reg, prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	udelay(20);

	/* de-assert cluster resets */
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg |= CPUCFG_CX_RST_CTRL_DBG_SOC_RST;
	reg |= CPUCFG_CX_RST_CTRL_H_RST;
	reg |= CPUCFG_CX_RST_CTRL_L2_RST;
	if (sunxi_core_is_cortex_a15(0, cluster))
		reg |= CPUCFG_CX_RST_CTRL_CX_RST_ALL;
	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));

	/* de-assert ACINACTM */
	reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));
	reg &= ~CPUCFG_CX_CTRL_REG1_ACINACTM;
	writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));

	return 0;
}

/*
 * This bit is shared between the initial nocache_trampoline call to
 * enable CCI-400 and proper cluster cache disable before power down.
 */
static void sunxi_cluster_cache_disable_without_axi(void)
{
	if (read_cpuid_part() == ARM_CPU_PART_CORTEX_A15) {
		/*
		 * On the Cortex-A15 we need to disable
		 * L2 prefetching before flushing the cache.
		 */
		asm volatile(
		"mcr	p15, 1, %0, c15, c0, 3\n"
		"isb\n"
		"dsb"
		: : "r" (0x400));
	}

	/* Flush all cache levels for this cluster. */
	v7_exit_coherency_flush(all);

	/*
	 * Disable cluster-level coherency by masking
	 * incoming snoops and DVM messages:
	 */
	cci_disable_port_by_cpu(read_cpuid_mpidr());
}

static int sunxi_mc_smp_cpu_table[SUNXI_NR_CLUSTERS][SUNXI_CPUS_PER_CLUSTER];
int sunxi_mc_smp_first_comer;

static DEFINE_SPINLOCK(boot_lock);

static bool sunxi_mc_smp_cluster_is_down(unsigned int cluster)
{
	int i;

	for (i = 0; i < SUNXI_CPUS_PER_CLUSTER; i++)
		if (sunxi_mc_smp_cpu_table[cluster][i])
			return false;
	return true;
}

static void sunxi_mc_smp_secondary_init(unsigned int cpu)
{
	/* Clear hotplug support magic flags for cpu0 */
	if (cpu == 0)
		sunxi_cpu0_hotplug_support_set(false);
}

static int sunxi_mc_smp_boot_secondary(unsigned int l_cpu, struct task_struct *idle)
{
	unsigned int mpidr, cpu, cluster;
	int ret;

	mpidr = cpu_logical_map(l_cpu);
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	if (!cpucfg_base)
		return -ENODEV;
	if (cluster >= SUNXI_NR_CLUSTERS || cpu >= SUNXI_CPUS_PER_CLUSTER)
		return -EINVAL;

	spin_lock_irq(&boot_lock);

	if (sunxi_mc_smp_cpu_table[cluster][cpu])
		goto out;

	if (sunxi_mc_smp_cluster_is_down(cluster)) {
		sunxi_mc_smp_first_comer = true;
		ret = sunxi_cluster_powerup(cluster);
		if (ret) {
			/*
			 * Leave the table alone so a later attempt retries the
			 * power-up instead of assuming this CPU is already up.
			 */
			spin_unlock_irq(&boot_lock);
			return ret;
		}
	} else {
		sunxi_mc_smp_first_comer = false;
	}

	/* This is read by incoming CPUs with their cache and MMU disabled */
	sync_cache_w(&sunxi_mc_smp_first_comer);
	sunxi_cpu_powerup(cpu, cluster);

out:
	sunxi_mc_smp_cpu_table[cluster][cpu]++;
	spin_unlock_irq(&boot_lock);

	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static void sunxi_cluster_cache_disable(void)
{
	unsigned int cluster = MPIDR_AFFINITY_LEVEL(read_cpuid_mpidr(), 1);
	u32 reg;

	pr_debug("%s: cluster %u\n", __func__, cluster);

	sunxi_cluster_cache_disable_without_axi();

	/* last man standing, assert ACINACTM */
	reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));
	reg |= CPUCFG_CX_CTRL_REG1_ACINACTM;
	writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));
}

static void sunxi_mc_smp_cpu_die(unsigned int l_cpu)
{
	unsigned int mpidr, cpu, cluster;
	bool last_man;

	mpidr = cpu_logical_map(l_cpu);
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	pr_debug("%s: cluster %u cpu %u\n", __func__, cluster, cpu);

	spin_lock(&boot_lock);
	sunxi_mc_smp_cpu_table[cluster][cpu]--;
	if (sunxi_mc_smp_cpu_table[cluster][cpu] == 1) {
		/* A power_up request went ahead of us. */
		pr_debug("%s: aborting due to a power up request\n",
			 __func__);
		spin_unlock(&boot_lock);
		return;
	} else if (sunxi_mc_smp_cpu_table[cluster][cpu] > 1) {
		pr_err("Cluster %d CPU%d boots multiple times\n",
		       cluster, cpu);
		BUG();
	}

	last_man = sunxi_mc_smp_cluster_is_down(cluster);
	spin_unlock(&boot_lock);

	gic_cpu_if_down(0);
	if (last_man)
		sunxi_cluster_cache_disable();
	else
		v7_exit_coherency_flush(louis);

	for (;;)
		wfi();
}

static int sunxi_cpu_powerdown(unsigned int cpu, unsigned int cluster)
{
	u32 reg;
	int gating_bit = cpu;

	pr_debug("%s: cluster %u cpu %u\n", __func__, cluster, cpu);
	if (cpu >= SUNXI_CPUS_PER_CLUSTER || cluster >= SUNXI_NR_CLUSTERS)
		return -EINVAL;

	if (is_a83t && cpu == 0)
		gating_bit = 4;

	/* gate processor power */
	reg = readl(prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	reg |= PRCM_PWROFF_GATING_REG_CORE(gating_bit);
	writel(reg, prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	udelay(20);

	/* close power switch */
	sunxi_cpu_power_switch_set(cpu, cluster, false);

	return 0;
}

static int sunxi_cluster_powerdown(unsigned int cluster)
{
	u32 reg;

	pr_debug("%s: cluster %u\n", __func__, cluster);
	if (cluster >= SUNXI_NR_CLUSTERS)
		return -EINVAL;

	/* assert cluster resets or system will hang */
	pr_debug("%s: assert cluster reset\n", __func__);
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg &= ~CPUCFG_CX_RST_CTRL_DBG_SOC_RST;
	reg &= ~CPUCFG_CX_RST_CTRL_H_RST;
	reg &= ~CPUCFG_CX_RST_CTRL_L2_RST;
	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));

	/* gate cluster power */
	pr_debug("%s: gate cluster power\n", __func__);
	reg = readl(prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	if (is_a83t)
		reg |= PRCM_PWROFF_GATING_REG_CLUSTER_SUN8I;
	else
		reg |= PRCM_PWROFF_GATING_REG_CLUSTER_SUN9I;
	writel(reg, prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	udelay(20);

	return 0;
}

static int sunxi_mc_smp_cpu_kill(unsigned int l_cpu)
{
	unsigned int mpidr, cpu, cluster;
	unsigned int tries, count;
	int ret = 0;
	u32 reg;

	mpidr = cpu_logical_map(l_cpu);
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	/* This should never happen */
	if (WARN_ON(cluster >= SUNXI_NR_CLUSTERS ||
		    cpu >= SUNXI_CPUS_PER_CLUSTER))
		return 0;

	/* wait for CPU core to die and enter WFI */
	count = TIMEOUT_USEC / POLL_USEC;
	spin_lock_irq(&boot_lock);
	for (tries = 0; tries < count; tries++) {
		spin_unlock_irq(&boot_lock);
		usleep_range(POLL_USEC / 2, POLL_USEC);
		spin_lock_irq(&boot_lock);

		/*
		 * If the user turns off a bunch of cores at the same
		 * time, the kernel might call cpu_kill before some of
		 * them are ready. This is because boot_lock serializes
		 * both cpu_die and cpu_kill callbacks. Either one could
		 * run first. We should wait for cpu_die to complete.
		 */
		if (sunxi_mc_smp_cpu_table[cluster][cpu])
			continue;

		reg = readl(cpucfg_base + CPUCFG_CX_STATUS(cluster));
		if (reg & CPUCFG_CX_STATUS_STANDBYWFI(cpu))
			break;
	}

	if (tries >= count) {
		ret = ETIMEDOUT;
		goto out;
	}

	/* power down CPU core */
	sunxi_cpu_powerdown(cpu, cluster);

	if (!sunxi_mc_smp_cluster_is_down(cluster))
		goto out;

	/* wait for cluster L2 WFI */
	ret = readl_poll_timeout(cpucfg_base + CPUCFG_CX_STATUS(cluster), reg,
				 reg & CPUCFG_CX_STATUS_STANDBYWFIL2,
				 POLL_USEC, TIMEOUT_USEC);
	if (ret) {
		/*
		 * Ignore timeout on the cluster. Leaving the cluster on
		 * will not affect system execution, just use a bit more
		 * power. But returning an error here will only confuse
		 * the user as the CPU has already been shutdown.
		 */
		ret = 0;
		goto out;
	}

	/* Power down cluster */
	sunxi_cluster_powerdown(cluster);

out:
	spin_unlock_irq(&boot_lock);
	pr_debug("%s: cluster %u cpu %u powerdown: %d\n",
		 __func__, cluster, cpu, ret);
	return !ret;
}

static bool sunxi_mc_smp_cpu_can_disable(unsigned int cpu)
{
	/* CPU0 hotplug not handled for sun8i-a83t */
	if (is_a83t)
		if (cpu == 0)
			return false;
	return true;
}
#endif

static const struct smp_operations sunxi_mc_smp_smp_ops __initconst = {
	.smp_secondary_init	= sunxi_mc_smp_secondary_init,
	.smp_boot_secondary	= sunxi_mc_smp_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= sunxi_mc_smp_cpu_die,
	.cpu_kill		= sunxi_mc_smp_cpu_kill,
	.cpu_can_disable	= sunxi_mc_smp_cpu_can_disable,
#endif
};

static bool __init sunxi_mc_smp_cpu_table_init(void)
{
	unsigned int mpidr, cpu, cluster;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	if (cluster >= SUNXI_NR_CLUSTERS || cpu >= SUNXI_CPUS_PER_CLUSTER) {
		pr_err("%s: boot CPU is out of bounds!\n", __func__);
		return false;
	}
	sunxi_mc_smp_cpu_table[cluster][cpu] = 1;
	return true;
}

/*
 * Adapted from arch/arm/common/mc_smp_entry.c
 *
 * We need the trampoline code to enable CCI-400 on the first cluster
 */
typedef typeof(cpu_reset) phys_reset_t;

static int __init nocache_trampoline(unsigned long __unused)
{
	phys_reset_t phys_reset;

	setup_mm_for_reboot();
	sunxi_cluster_cache_disable_without_axi();

	phys_reset = (phys_reset_t)(unsigned long)__pa_symbol(cpu_reset);
	phys_reset(__pa_symbol(sunxi_mc_smp_resume), false);
	BUG();
}

static int __init sunxi_mc_smp_loopback(void)
{
	int ret;

	/*
	 * We're going to soft-restart the current CPU through the
	 * low-level MCPM code by leveraging the suspend/resume
	 * infrastructure. Let's play it safe by using cpu_pm_enter()
	 * in case the CPU init code path resets the VFP or similar.
	 */
	sunxi_mc_smp_first_comer = true;
	local_irq_disable();
	local_fiq_disable();
	ret = cpu_pm_enter();
	if (!ret) {
		ret = cpu_suspend(0, nocache_trampoline);
		cpu_pm_exit();
	}
	local_fiq_enable();
	local_irq_enable();
	sunxi_mc_smp_first_comer = false;

	return ret;
}

/*
 * This holds any device nodes that we requested resources for,
 * so that we may easily release resources in the error path.
 */
struct sunxi_mc_smp_nodes {
	struct device_node *prcm_node;
	struct device_node *cpucfg_node;
	struct device_node *sram_node;
	struct device_node *r_cpucfg_node;
};

/* This structure holds SoC-specific bits tied to an enable-method string. */
struct sunxi_mc_smp_data {
	const char *enable_method;
	int (*get_smp_nodes)(struct sunxi_mc_smp_nodes *nodes);
	bool is_a83t;
};

static void __init sunxi_mc_smp_put_nodes(struct sunxi_mc_smp_nodes *nodes)
{
	of_node_put(nodes->prcm_node);
	of_node_put(nodes->cpucfg_node);
	of_node_put(nodes->sram_node);
	of_node_put(nodes->r_cpucfg_node);
	memset(nodes, 0, sizeof(*nodes));
}

static int __init sun9i_a80_get_smp_nodes(struct sunxi_mc_smp_nodes *nodes)
{
	nodes->prcm_node = of_find_compatible_node(NULL, NULL,
						   "allwinner,sun9i-a80-prcm");
	if (!nodes->prcm_node) {
		pr_err("%s: PRCM not available\n", __func__);
		return -ENODEV;
	}

	nodes->cpucfg_node = of_find_compatible_node(NULL, NULL,
						     "allwinner,sun9i-a80-cpucfg");
	if (!nodes->cpucfg_node) {
		pr_err("%s: CPUCFG not available\n", __func__);
		return -ENODEV;
	}

	nodes->sram_node = of_find_compatible_node(NULL, NULL,
						   "allwinner,sun9i-a80-smp-sram");
	if (!nodes->sram_node) {
		pr_err("%s: Secure SRAM not available\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int __init sun8i_a83t_get_smp_nodes(struct sunxi_mc_smp_nodes *nodes)
{
	nodes->prcm_node = of_find_compatible_node(NULL, NULL,
						   "allwinner,sun8i-a83t-r-ccu");
	if (!nodes->prcm_node) {
		pr_err("%s: PRCM not available\n", __func__);
		return -ENODEV;
	}

	nodes->cpucfg_node = of_find_compatible_node(NULL, NULL,
						     "allwinner,sun8i-a83t-cpucfg");
	if (!nodes->cpucfg_node) {
		pr_err("%s: CPUCFG not available\n", __func__);
		return -ENODEV;
	}

	nodes->r_cpucfg_node = of_find_compatible_node(NULL, NULL,
						       "allwinner,sun8i-a83t-r-cpucfg");
	if (!nodes->r_cpucfg_node) {
		pr_err("%s: RCPUCFG not available\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static const struct sunxi_mc_smp_data sunxi_mc_smp_data[] __initconst = {
	{
		.enable_method	= "allwinner,sun9i-a80-smp",
		.get_smp_nodes	= sun9i_a80_get_smp_nodes,
	},
	{
		.enable_method	= "allwinner,sun8i-a83t-smp",
		.get_smp_nodes	= sun8i_a83t_get_smp_nodes,
		.is_a83t	= true,
	},
};

static int __init sunxi_mc_smp_init(void)
{
	struct sunxi_mc_smp_nodes nodes = { 0 };
	struct device_node *node;
	struct resource res;
	void __iomem *addr;
	int i, ret;

	/*
	 * Don't bother checking the "cpus" node, as an enable-method
	 * property in that node is undocumented.
	 */
	node = of_cpu_device_node_get(0);
	if (!node)
		return -ENODEV;

	/*
	 * We can't actually use the enable-method magic in the kernel.
	 * Our loopback / trampoline code uses the CPU suspend framework,
	 * which requires the identity mapping be available. It would not
	 * yet be available if we used the .init_cpus or .prepare_cpus
	 * callbacks in smp_operations, which we would use if we were to
	 * use CPU_METHOD_OF_DECLARE
	 */
	for (i = 0; i < ARRAY_SIZE(sunxi_mc_smp_data); i++) {
		ret = of_property_match_string(node, "enable-method",
					       sunxi_mc_smp_data[i].enable_method);
		if (ret >= 0)
			break;
	}

	of_node_put(node);
	if (ret < 0)
		return -ENODEV;

	is_a83t = sunxi_mc_smp_data[i].is_a83t;

	if (!sunxi_mc_smp_cpu_table_init())
		return -EINVAL;

	if (!cci_probed()) {
		pr_err("%s: CCI-400 not available\n", __func__);
		return -ENODEV;
	}

	/* Get needed device tree nodes */
	ret = sunxi_mc_smp_data[i].get_smp_nodes(&nodes);
	if (ret)
		goto err_put_nodes;

	/*
	 * Unfortunately we can not request the I/O region for the PRCM.
	 * It is shared with the PRCM clock.
	 */
	prcm_base = of_iomap(nodes.prcm_node, 0);
	if (!prcm_base) {
		pr_err("%s: failed to map PRCM registers\n", __func__);
		ret = -ENOMEM;
		goto err_put_nodes;
	}

	cpucfg_base = of_io_request_and_map(nodes.cpucfg_node, 0,
					    "sunxi-mc-smp");
	if (IS_ERR(cpucfg_base)) {
		ret = PTR_ERR(cpucfg_base);
		pr_err("%s: failed to map CPUCFG registers: %d\n",
		       __func__, ret);
		goto err_unmap_prcm;
	}

	if (is_a83t) {
		r_cpucfg_base = of_io_request_and_map(nodes.r_cpucfg_node,
						      0, "sunxi-mc-smp");
		if (IS_ERR(r_cpucfg_base)) {
			ret = PTR_ERR(r_cpucfg_base);
			pr_err("%s: failed to map R-CPUCFG registers\n",
			       __func__);
			goto err_unmap_release_cpucfg;
		}
	} else {
		sram_b_smp_base = of_io_request_and_map(nodes.sram_node, 0,
							"sunxi-mc-smp");
		if (IS_ERR(sram_b_smp_base)) {
			ret = PTR_ERR(sram_b_smp_base);
			pr_err("%s: failed to map secure SRAM\n", __func__);
			goto err_unmap_release_cpucfg;
		}
	}

	/* Configure CCI-400 for boot cluster */
	ret = sunxi_mc_smp_loopback();
	if (ret) {
		pr_err("%s: failed to configure boot cluster: %d\n",
		       __func__, ret);
		goto err_unmap_release_sram_rcpucfg;
	}

	/* We don't need the device nodes anymore */
	sunxi_mc_smp_put_nodes(&nodes);

	/* Set the hardware entry point address */
	if (is_a83t)
		addr = r_cpucfg_base + R_CPUCFG_CPU_SOFT_ENTRY_REG;
	else
		addr = prcm_base + PRCM_CPU_SOFT_ENTRY_REG;
	writel(__pa_symbol(sunxi_mc_smp_secondary_startup), addr);

	/* Actually enable multi cluster SMP */
	smp_set_ops(&sunxi_mc_smp_smp_ops);

	pr_info("sunxi multi cluster SMP support installed\n");

	return 0;

err_unmap_release_sram_rcpucfg:
	if (is_a83t) {
		iounmap(r_cpucfg_base);
		of_address_to_resource(nodes.r_cpucfg_node, 0, &res);
	} else {
		iounmap(sram_b_smp_base);
		of_address_to_resource(nodes.sram_node, 0, &res);
	}
	release_mem_region(res.start, resource_size(&res));
err_unmap_release_cpucfg:
	iounmap(cpucfg_base);
	of_address_to_resource(nodes.cpucfg_node, 0, &res);
	release_mem_region(res.start, resource_size(&res));
err_unmap_prcm:
	iounmap(prcm_base);
err_put_nodes:
	sunxi_mc_smp_put_nodes(&nodes);
	return ret;
}

early_initcall(sunxi_mc_smp_init);
