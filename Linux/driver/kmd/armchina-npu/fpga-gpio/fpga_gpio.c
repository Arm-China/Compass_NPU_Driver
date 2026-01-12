// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

/**
 * SoC: ArmChina internal Juno platform
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/io.h>
#include <linux/version.h>

struct fpga_gpio {
	struct gpio_chip chip;
	void __iomem *base;
	spinlock_t lock;
};
extern int gpiod_is_active_low(const struct gpio_desc *desc);
extern struct gpio_desc *gpio_to_desc(unsigned gpio);
static int fpga_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct fpga_gpio *priv = gpiochip_get_data(chip);
	u32 val;

	val = readl(priv->base + offset * 4);
	return !!(val & BIT(16));
}

static void fpga_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct fpga_gpio *priv = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&priv->lock, flags);
	val = readl(priv->base + offset * 4);

	if(value)
		val |= 0x10000;
	else
		val &= ~0x10000;
	writel(val, priv->base + offset * 4);
	spin_unlock_irqrestore(&priv->lock, flags);
	return;
}

static int fpga_gpio_direction_output(struct gpio_chip *chip,
				unsigned offset, int value)
{
	fpga_gpio_set(chip, offset, value);
	return 0;
}

static int fpga_gpio_probe(struct platform_device *pdev)
{
	struct fpga_gpio *priv;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	spin_lock_init(&priv->lock);

	priv->chip.label = "fpga-gpio";
	priv->chip.parent = &pdev->dev;
	priv->chip.owner = THIS_MODULE;
	priv->chip.base = -1;
	priv->chip.ngpio = 2;
	priv->chip.get = fpga_gpio_get;
	priv->chip.set = fpga_gpio_set;
	priv->chip.direction_output = fpga_gpio_direction_output;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
	priv->chip.of_node = pdev->dev.of_node;
#endif
	ret = devm_gpiochip_add_data(&pdev->dev, &priv->chip, priv);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add GPIO chip: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "FPGA GPIO controller registered\n");
	return 0;
}

static const struct of_device_id fpga_gpio_of_match[] = {
	{ .compatible = "armchina,fpga-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, fpga_gpio_of_match);

static struct platform_driver fpga_gpio_driver = {
	.probe = fpga_gpio_probe,
	.driver = {
		.name = "fpga-gpio",
		.of_match_table = fpga_gpio_of_match,
	},
};
module_platform_driver(fpga_gpio_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Michael Chen");
MODULE_DESCRIPTION("ArmChina fpga gpio driver sample");