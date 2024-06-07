// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/dma-buf-map.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/of.h>

static struct platform_device *importer_dev;

static int importer_test(struct dma_buf *dmabuf)
{
	const char *magic = "This is string filled by kernel module!";
	struct dma_buf_attachment *attachment;
	struct sg_table *table;
	unsigned int reg_addr, reg_size;
	void *vaddr = NULL;
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 11, 0) && \
    LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
	struct dma_buf_map map;
#endif

	printk(KERN_INFO "enter %s\n", __func__);
	dev_set_name(&importer_dev->dev, "importer");

	attachment = dma_buf_attach(dmabuf, &importer_dev->dev);
	printk(KERN_INFO "after dma_buf_attach\n");
	table = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	printk(KERN_INFO "after dma_buf_map_attachment\n");

	reg_addr = sg_dma_address(table->sgl);
	reg_size = sg_dma_len(table->sgl);
	pr_info("reg_addr = 0x%08x, reg_size = 0x%08x\n", reg_addr, reg_size);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
	vaddr = dma_buf_vmap(dmabuf);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
	dma_buf_vmap(dmabuf, &map);
	vaddr = map.vaddr;
#endif
	memcpy(vaddr, magic, strlen(magic) + 1);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
	dma_buf_vunmap(dmabuf, vaddr);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
	dma_buf_vunmap(dmabuf, &map);
#endif
	dma_buf_unmap_attachment(attachment, table, DMA_BIDIRECTIONAL);
	dma_buf_detach(dmabuf, attachment);
	return 0;
}

int importer_open(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "enter %s\n", __func__);
	return 0;
}

int importer_release(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "enter %s\n", __func__);
	return 0;
}

static long importer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int fd = 0;
	struct dma_buf *dmabuf = NULL;
	int len = 0;

	len = copy_from_user(&fd, (void __user *)arg, sizeof(int));
	printk(KERN_INFO "enter %s, dma_buf fd=%d\n", __func__, fd);

	dmabuf = dma_buf_get(fd);
	importer_test(dmabuf);
	dma_buf_put(dmabuf);
	return 0;
}

static struct file_operations importer_fops = {
	.owner	= THIS_MODULE,
	.open = importer_open,
	.release = importer_release,
	.unlocked_ioctl	= importer_ioctl,
};

static struct miscdevice mdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "importer",
	.fops = &importer_fops,
};

static int importer_probe(struct platform_device *p_dev)
{
	pr_info("zhouyi importer probe.\n");
	importer_dev = p_dev;
	if (misc_register(&mdev)) {
		dev_err(&p_dev->dev, "zhouyi importer fail to register misc dev.\n");
		return -1;
	}
	return 0;
}

static int importer_remove(struct platform_device *p_dev)
{
	pr_info("zhouyi importer removed.\n");
	misc_deregister(&mdev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id importer_of_match[] = {
	{
		.compatible = "armchina,importer",
	},
	{}
};

MODULE_DEVICE_TABLE(of, importer_of_match);
#endif

static struct platform_driver importer_platform_driver = {
	.probe = importer_probe,
	.remove = importer_remove,
	.driver = {
		.name = "armchina-importer",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(importer_of_match),
#endif
	},
};

module_platform_driver(importer_platform_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ArmChina Zhouyi dma buffer test driver");
