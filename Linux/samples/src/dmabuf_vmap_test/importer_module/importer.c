// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/dma-buf-map.h>

static int importer_test(struct dma_buf *dmabuf)
{
	int ret = 0;
	const char *magic = "This is string filled by kernel module!";
	struct dma_buf_map db_map = DMA_BUF_MAP_INIT_VADDR(0);

	// write one line in dma_buf
	ret = dma_buf_vmap(dmabuf, &db_map);
	memcpy(db_map.vaddr, magic, strlen(magic) + 1);
	dma_buf_vunmap(dmabuf, &db_map);
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
MODULE_LICENSE("GPL");