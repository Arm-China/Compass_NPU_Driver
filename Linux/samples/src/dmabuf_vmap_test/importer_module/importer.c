// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
#include <linux/iosys-map.h>
#else
#include <linux/dma-buf-map.h>
#endif

static int importer_test(struct dma_buf *dmabuf)
{
	int ret = 0;
	const char *magic = "This is string filled by kernel module!";
#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
	struct iosys_map db_map = IOSYS_MAP_INIT_VADDR(0);
#else
	struct dma_buf_map db_map = DMA_BUF_MAP_INIT_VADDR(0);
#endif

	// write one line in dma_buf
	ret = dma_buf_vmap(dmabuf, &db_map);
	memcpy(db_map.vaddr, magic, strlen(magic) + 1);
	dma_buf_vunmap(dmabuf, &db_map);
	return 0;
}

static int importer_open(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "enter %s\n", __func__);
	return 0;
}

static int importer_release(struct inode *inode, struct file *filp)
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
	.mode = 0666,
};

static int __init importer_init(void)
{
	printk(KERN_INFO "enter %s\n", __func__);
	return misc_register(&mdev);
}

static void __exit importer_exit(void)
{
	printk(KERN_INFO "exit %s\n", __func__);
	misc_deregister(&mdev);
}

module_init(importer_init);
module_exit(importer_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ArmChina Zhouyi dma buffer test driver");
#if KERNEL_VERSION(5, 4, 0) < LINUX_VERSION_CODE
MODULE_IMPORT_NS(DMA_BUF);
#endif