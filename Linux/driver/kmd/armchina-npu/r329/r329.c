// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

/**
 * SoC: Allwinner R329 platform
 */

#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk.h>
#include "armchina_aipu_soc.h"

#define R329_AIPU_CLOCK_RATE   (600 * SZ_1M)

static struct aipu_soc r329 = {
	.priv = NULL,
};

static int r329_enable_clk(struct device *dev, struct aipu_soc *soc)
{
	struct clk *clk_pll_aipu = NULL;
	struct clk *clk_aipu = NULL;
	struct clk *clk_aipu_slv = NULL;
	struct device_node *dev_node = NULL;

	WARN_ON(!dev);
	dev_node = dev->of_node;

	clk_pll_aipu = of_clk_get(dev_node, 0);
	if (IS_ERR_OR_NULL(clk_pll_aipu)) {
		dev_err(dev, "clk_pll_aipu get failed\n");
		return PTR_ERR(clk_pll_aipu);
	}

	clk_aipu = of_clk_get(dev_node, 1);
	if (IS_ERR_OR_NULL(clk_aipu)) {
		dev_err(dev, "clk_aipu get failed\n");
		return PTR_ERR(clk_aipu);
	}

	clk_aipu_slv = of_clk_get(dev_node, 2);
	if (IS_ERR_OR_NULL(clk_aipu_slv)) {
		dev_err(dev, "clk_pll_aipu get failed\n");
		return PTR_ERR(clk_aipu_slv);
	}

	if (clk_set_parent(clk_aipu, clk_pll_aipu)) {
		dev_err(dev, "set clk_aipu parent fail\n");
		return -EBUSY;
	}

	if (clk_set_rate(clk_aipu, R329_AIPU_CLOCK_RATE)) {
		dev_err(dev, "set clk_aipu rate fail\n");
		return -EBUSY;
	}

	if (clk_prepare_enable(clk_aipu_slv)) {
		dev_err(dev, "clk_aipu_slv enable failed\n");
		return -EBUSY;
	}

	if (clk_prepare_enable(clk_aipu)) {
		dev_err(dev, "clk_aipu enable failed\n");
		return -EBUSY;
	}

	dev_info(dev, "enable r329 AIPU clock done\n");
	return 0;
}

static int r329_disable_clk(struct device *dev, struct aipu_soc *soc)
{
	struct clk *clk_aipu = NULL;
	struct clk *clk_aipu_slv = NULL;
	struct device_node *dev_node = NULL;

	WARN_ON(!dev);
	dev_node = dev->of_node;

	clk_aipu_slv = of_clk_get(dev_node, 2);
	if (clk_aipu_slv)
		clk_disable_unprepare(clk_aipu_slv);

	clk_aipu = of_clk_get(dev_node, 1);
	if (clk_aipu)
		clk_disable_unprepare(clk_aipu);

	dev_info(dev, "disable r329 AIPU clock done\n");
	return 0;
}

static struct aipu_soc_operations r329_ops = {
	.start_bw_profiling = NULL,
	.stop_bw_profiling = NULL,
	.read_profiling_reg = NULL,
	.enable_clk = r329_enable_clk,
	.disable_clk = r329_disable_clk,
	.is_clk_enabled = NULL,
	.is_aipu_irq = NULL,
	.soc_pm_runtime_get_sync = NULL,
	.soc_pm_runtime_put = NULL,
};

static int r329_probe(struct platform_device *p_dev)
{
	r329_enable_clk(&p_dev->dev, &r329);
	return armchina_aipu_probe(p_dev, &r329, &r329_ops);
}

static int r329_remove(struct platform_device *p_dev)
{
	int ret = armchina_aipu_remove(p_dev);

	r329_disable_clk(&p_dev->dev, &r329);
	return ret;
}

static int r329_suspend(struct platform_device *p_dev, pm_message_t state)
{
	return armchina_aipu_suspend(p_dev, state);
}

static int r329_resume(struct platform_device *p_dev)
{
	return armchina_aipu_resume(p_dev);
}

#ifdef CONFIG_OF
static const struct of_device_id aipu_of_match[] = {
	{
		.compatible = "armchina,zhouyi-v1",
	},
	{
		.compatible = "armchina,zhouyi",
	},
	{
		.compatible = "armchina,zhouyiv1aipu",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, aipu_of_match);
#endif

static struct platform_driver aipu_platform_driver = {
	.probe = r329_probe,
	.remove = r329_remove,
	.suspend = r329_suspend,
	.resume  = r329_resume,
	.driver = {
		.name = "armchina",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(aipu_of_match),
#endif
	},
};

module_platform_driver(aipu_platform_driver);
MODULE_LICENSE("GPL v2");
