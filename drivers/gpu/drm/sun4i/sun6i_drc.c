// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 Free Electrons
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "sun6i_drc.h"

/*
 * The A80's DRC sits between the display backend and the TCON and does not
 * pass anything through until it is told how big the picture is and enabled.
 * On the A31, where this driver was written, it happens to come out of reset
 * transparent, so nothing here ever wrote a register.
 *
 * Measured on a Tronsmart Draco AW80: with these two writes the backend's
 * layer reaches HDMI, without them the DRC emits nothing at all.
 */
#define SUN6I_DRC_GNECTL		0x00
#define SUN6I_DRC_GNECTL_EN			BIT(0)
#define SUN6I_DRC_GNECTL_MOD(m)			(((m) & 3) << 8)
#define SUN6I_DRC_DRCSIZE		0x04
#define SUN6I_DRC_DRCSIZE_WIDTH(w)		(((w) - 1) & 0xfff)
#define SUN6I_DRC_DRCSIZE_HEIGHT(h)		((((h) - 1) & 0xfff) << 16)
#define SUN6I_DRC_DRCCTL		0x10
#define SUN6I_DRC_DRCCTL_DB_EN			BIT(0)
#define SUN6I_DRC_DRCCTL_DBRDY_CTL		BIT(1)
#define SUN6I_DRC_CSC			0xc0
#define SUN6I_DRC_CSC_UNITY			0x400

/*
 * The DRC ends in a colour space converter, and it comes out of reset
 * holding a BT.709 YUV->RGB matrix. The backend hands it RGB, so without
 * an identity matrix here every colour is wrong - the picture appears, but
 * red-dominant, because RGB is being run through a YUV conversion.
 *
 * Twelve words: three rows of (coefficient x3, constant), 0x400 is 1.0.
 */
static const u32 sun6i_drc_csc_identity[12] = {
	SUN6I_DRC_CSC_UNITY, 0, 0, 0,
	0, SUN6I_DRC_CSC_UNITY, 0, 0,
	0, 0, SUN6I_DRC_CSC_UNITY, 0,
};

struct sun6i_drc {
	struct clk		*bus_clk;
	struct clk		*mod_clk;
	struct clk		*ram_clk;
	struct reset_control	*reset;
	void __iomem		*regs;
	bool			needs_setup;
};

static int sun6i_drc_bind(struct device *dev, struct device *master,
			 void *data)
{
	struct sun6i_drc *drc;
	int ret;

	drc = devm_kzalloc(dev, sizeof(*drc), GFP_KERNEL);
	if (!drc)
		return -ENOMEM;
	dev_set_drvdata(dev, drc);

	drc->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(drc->reset)) {
		dev_err(dev, "Couldn't get our reset line\n");
		return PTR_ERR(drc->reset);
	}

	ret = reset_control_deassert(drc->reset);
	if (ret) {
		dev_err(dev, "Couldn't deassert our reset line\n");
		return ret;
	}

	drc->bus_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(drc->bus_clk)) {
		dev_err(dev, "Couldn't get our bus clock\n");
		ret = PTR_ERR(drc->bus_clk);
		goto err_assert_reset;
	}
	clk_prepare_enable(drc->bus_clk);

	drc->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR(drc->mod_clk)) {
		dev_err(dev, "Couldn't get our mod clock\n");
		ret = PTR_ERR(drc->mod_clk);
		goto err_disable_bus_clk;
	}

	ret = clk_set_rate_exclusive(drc->mod_clk, 300000000);
	if (ret) {
		dev_err(dev, "Couldn't set the module clock frequency\n");
		goto err_disable_bus_clk;
	}

	clk_prepare_enable(drc->mod_clk);

	/*
	 * The binding has always described three clocks for this block and
	 * this driver has only ever asked for two. sun4i_backend.c gets its
	 * "ram" clock right; the DRC never has.
	 */
	drc->ram_clk = devm_clk_get_optional(dev, "ram");
	if (!IS_ERR(drc->ram_clk))
		clk_prepare_enable(drc->ram_clk);

	drc->needs_setup = of_device_is_compatible(dev->of_node,
						   "allwinner,sun9i-a80-drc");
	if (drc->needs_setup) {
		drc->regs = devm_platform_ioremap_resource(
				to_platform_device(dev), 0);
		if (IS_ERR(drc->regs)) {
			dev_err(dev, "Couldn't map the DRC registers\n");
			drc->regs = NULL;
			drc->needs_setup = false;
		}
	}

	return 0;

err_disable_bus_clk:
	clk_disable_unprepare(drc->bus_clk);
err_assert_reset:
	reset_control_assert(drc->reset);
	return ret;
}


/*
 * Called by the display engine once it knows the mode. The DRC has no way
 * of learning the picture size on its own - it is not a DRM component with
 * a mode of its own - so the engine hands it down the of_graph link.
 *
 * Only the size and the enable are needed. The gain table, the bilateral
 * filter coefficients, the CSC, the histogram window and the mode field
 * were all tried and none of them changes the output.
 */
void sun6i_drc_engine_mode_set(struct device_node *engine_node,
			       u32 width, u32 height)
{
	struct device_node *ep, *remote;
	struct platform_device *pdev;
	struct sun6i_drc *drc;
	int i;

	/* the engine's output port (1) leads to the DRC on sun9i */
	ep = of_graph_get_endpoint_by_regs(engine_node, 1, -1);
	if (!ep)
		return;

	remote = of_graph_get_remote_port_parent(ep);
	of_node_put(ep);
	if (!remote)
		return;

	pdev = of_find_device_by_node(remote);
	of_node_put(remote);
	if (!pdev)
		return;

	drc = platform_get_drvdata(pdev);
	if (!drc || !drc->needs_setup || !drc->regs)
		goto out;

	writel(SUN6I_DRC_DRCSIZE_HEIGHT(height) |
	       SUN6I_DRC_DRCSIZE_WIDTH(width),
	       drc->regs + SUN6I_DRC_DRCSIZE);

	for (i = 0; i < ARRAY_SIZE(sun6i_drc_csc_identity); i++)
		writel(sun6i_drc_csc_identity[i],
		       drc->regs + SUN6I_DRC_CSC + i * 4);
	writel(SUN6I_DRC_GNECTL_MOD(2) | SUN6I_DRC_GNECTL_EN,
	       drc->regs + SUN6I_DRC_GNECTL);

	/*
	 * The block is double buffered: none of the above becomes active
	 * until it is latched, which is what the vendor's
	 * DRC_EBIOS_Drc_Cfg_Rdy() does - dbrdy_ctl first, then db_en.
	 * The registers read back correctly without this and still have
	 * no effect whatsoever.
	 */
	writel(SUN6I_DRC_DRCCTL_DBRDY_CTL, drc->regs + SUN6I_DRC_DRCCTL);
	writel(SUN6I_DRC_DRCCTL_DBRDY_CTL | SUN6I_DRC_DRCCTL_DB_EN,
	       drc->regs + SUN6I_DRC_DRCCTL);

out:
	put_device(&pdev->dev);
}
EXPORT_SYMBOL_GPL(sun6i_drc_engine_mode_set);

static void sun6i_drc_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct sun6i_drc *drc = dev_get_drvdata(dev);

	clk_rate_exclusive_put(drc->mod_clk);
	clk_disable_unprepare(drc->mod_clk);
	clk_disable_unprepare(drc->bus_clk);
	reset_control_assert(drc->reset);
}

static const struct component_ops sun6i_drc_ops = {
	.bind	= sun6i_drc_bind,
	.unbind	= sun6i_drc_unbind,
};

static int sun6i_drc_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &sun6i_drc_ops);
}

static void sun6i_drc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &sun6i_drc_ops);
}

static const struct of_device_id sun6i_drc_of_table[] = {
	{ .compatible = "allwinner,sun6i-a31-drc" },
	{ .compatible = "allwinner,sun6i-a31s-drc" },
	{ .compatible = "allwinner,sun8i-a23-drc" },
	{ .compatible = "allwinner,sun8i-a33-drc" },
	{ .compatible = "allwinner,sun9i-a80-drc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun6i_drc_of_table);

static struct platform_driver sun6i_drc_platform_driver = {
	.probe		= sun6i_drc_probe,
	.remove		= sun6i_drc_remove,
	.driver		= {
		.name		= "sun6i-drc",
		.of_match_table	= sun6i_drc_of_table,
	},
};
module_platform_driver(sun6i_drc_platform_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Allwinner A31 Dynamic Range Control (DRC) Driver");
MODULE_LICENSE("GPL");
