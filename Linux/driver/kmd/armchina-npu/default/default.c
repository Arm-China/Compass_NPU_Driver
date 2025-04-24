// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

/**
 * SoC: ArmChina internal Juno platform
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include "armchina_aipu_soc.h"

static struct aipu_soc default_soc = {
	.priv = NULL,
};

static struct aipu_soc_operations default_ops = {
	.start_bw_profiling = NULL,
	.stop_bw_profiling = NULL,
	.read_profiling_reg = NULL,
	.enable_clk = NULL,
	.disable_clk = NULL,
	.is_clk_enabled = NULL,
	.is_aipu_irq = NULL,
	.soc_pm_runtime_get_sync = NULL,
	.soc_pm_runtime_put = NULL,
};

static int default_probe(struct platform_device *p_dev)
{
	return armchina_aipu_probe(p_dev, &default_soc, &default_ops);
}

static int default_remove(struct platform_device *p_dev)
{
	return armchina_aipu_remove(p_dev);
}

static int default_suspend(struct platform_device *p_dev, pm_message_t state)
{
	return armchina_aipu_suspend(p_dev, state);
}

static int default_resume(struct platform_device *p_dev)
{
	return armchina_aipu_resume(p_dev);
}

#ifdef CONFIG_OF
static const struct of_device_id aipu_of_match[] = {
	{
		.compatible = "armchina,zhouyi-v1",
	},
	{
		.compatible = "armchina,zhouyi-v2",
	},
	{
		.compatible = "armchina,zhouyi-v3",
	},
	{
		.compatible = "armchina,zhouyi-v3_1",
	},
	{
		.compatible = "armchina,zhouyi",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, aipu_of_match);
#endif

static struct platform_driver aipu_platform_driver = {
	.probe = default_probe,
	.remove = default_remove,
	.suspend = default_suspend,
	.resume  = default_resume,
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
MODULE_AUTHOR("Dejia Shang");
MODULE_AUTHOR("Toby Huang");
MODULE_DESCRIPTION("ArmChina Zhouyi AI accelerator driver");
