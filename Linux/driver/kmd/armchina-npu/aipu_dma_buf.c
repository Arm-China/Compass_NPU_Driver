// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/version.h>
#include <linux/module.h>
#include "aipu_dma_buf.h"

#if KERNEL_VERSION(4, 19, 0) > LINUX_VERSION_CODE
static int aipu_dma_buf_attach(struct dma_buf *dmabuf, struct device *dev,
			       struct dma_buf_attachment *attach)
#else
static int aipu_dma_buf_attach(struct dma_buf *dmabuf, struct dma_buf_attachment *attach)
#endif
{
	return 0;
}

static void aipu_dma_buf_detach(struct dma_buf *dmabuf, struct dma_buf_attachment *attach)
{
}

static struct sg_table *aipu_map_dma_buf(struct dma_buf_attachment *attach,
					 enum dma_data_direction dir)
{
	int ret = 0;
	struct sg_table *sgt = NULL;
	struct aipu_dma_buf_priv *priv = (struct aipu_dma_buf_priv *)attach->dmabuf->priv;
	struct device *npu = priv->mm->dev;

	if (priv->sgt)
		return priv->sgt;

	sgt = devm_kmalloc(npu, sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return NULL;

	ret = dma_get_sgtable_attrs(attach->dev, sgt, priv->va, priv->dma_pa, priv->bytes, 0);
	if (ret < 0) {
		dev_err(npu, "failed to get scatterlist from DMA API\n");
		goto fail;
	}

#if KERNEL_VERSION(5, 8, 0) > LINUX_VERSION_CODE
	ret = dma_map_sg(attach->dev, sgt->sgl, sgt->nents, dir);
#else
	ret = dma_map_sgtable(attach->dev, sgt, dir, 0);
#endif
	if (ret) {
		dev_err(npu, "failed to map sgtable for the attached dev\n");
		goto fail;
	}

	priv->sgt = sgt;
	return sgt;

fail:
	devm_kfree(npu, sgt);
	return NULL;
}

static void aipu_unmap_dma_buf(struct dma_buf_attachment *attach, struct sg_table *st,
			       enum dma_data_direction dir)
{
}

static int aipu_dma_buf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	int ret = 0;
	unsigned long vm_pgoff = 0;
	struct aipu_dma_buf_priv *priv = (struct aipu_dma_buf_priv *)dmabuf->priv;

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = priv->dev_pa >> PAGE_SHIFT;
	ret = aipu_mm_mmap_buf(priv->mm, vma, NULL);
	vma->vm_pgoff = vm_pgoff;
	return ret;
}

#if KERNEL_VERSION(5, 11, 0) > LINUX_VERSION_CODE
static void *aipu_dma_vmap(struct dma_buf *dmabuf)
#elif KERNEL_VERSION(5, 18, 0) > LINUX_VERSION_CODE
static int aipu_dma_vmap(struct dma_buf *dmabuf, struct dma_buf_map *map)
#else
static int aipu_dma_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
#endif
{
	struct aipu_dma_buf_priv *priv = (struct aipu_dma_buf_priv *)dmabuf->priv;
#if KERNEL_VERSION(5, 11, 0) > LINUX_VERSION_CODE
	return priv->va;
#else
	map->is_iomem = false;
	map->vaddr = priv->va;
	return 0;
#endif
}

#if KERNEL_VERSION(5, 11, 0) > LINUX_VERSION_CODE
static void aipu_dma_vunmap(struct dma_buf *dmabuf, void *vaddr)
#elif KERNEL_VERSION(5, 18, 0) > LINUX_VERSION_CODE
static void aipu_dma_vunmap(struct dma_buf *dmabuf, struct dma_buf_map *map)
#else
static void aipu_dma_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
#endif
{
}

static void aipu_dma_release(struct dma_buf *dmabuf)
{
}

static struct dma_buf_ops aipu_dma_buf_ops = {
	.attach = aipu_dma_buf_attach,
	.detach = aipu_dma_buf_detach,
	.map_dma_buf = aipu_map_dma_buf,
	.unmap_dma_buf = aipu_unmap_dma_buf,
	.mmap   = aipu_dma_buf_mmap,
	.vmap   = aipu_dma_vmap,
	.vunmap = aipu_dma_vunmap,
	.release = aipu_dma_release,
};

int aipu_alloc_dma_buf(struct aipu_memory_manager *mm, struct aipu_dma_buf_request *request)
{
	int ret = 0;
	struct aipu_buf_request inter_req;
	struct aipu_dma_buf_priv *priv = NULL;
	char *va = NULL;
	struct dma_buf *dmabuf = NULL;

	DEFINE_DMA_BUF_EXPORT_INFO(exp);

	if (!mm || !request || !request->bytes)
		return -EINVAL;

	memset(&inter_req, 0, sizeof(struct aipu_buf_request));
	inter_req.bytes = request->bytes;
	inter_req.align_in_page = 1;
	inter_req.region = AIPU_BUF_REGION_DEFAULT;
	inter_req.asid = AIPU_BUF_ASID_0;

	ret = aipu_mm_alloc(mm, &inter_req, NULL);
	if (ret)
		return ret;

	va = aipu_mm_get_va(mm, inter_req.desc.pa);
	if (!va) {
		ret = -EFAULT;
		goto fail;
	}

	priv = devm_kzalloc(mm->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto fail;
	}

	priv->mm = mm;
	priv->dev_pa = inter_req.desc.pa;
	priv->dma_pa = inter_req.desc.pa;
	priv->bytes = inter_req.desc.bytes;
	priv->va = va;

	exp.ops = &aipu_dma_buf_ops;
	exp.size = inter_req.desc.bytes;
	exp.flags = O_RDWR | O_CLOEXEC;
	exp.priv = priv;

	dmabuf = dma_buf_export(&exp);
	if (!dmabuf) {
		ret = -EINVAL;
		goto fail;
	}

	request->fd = dma_buf_fd(dmabuf, exp.flags);
	return 0;

fail:
	if (priv)
		devm_kfree(mm->dev, priv);
	aipu_mm_free(mm, &inter_req.desc, NULL);
	return ret;
}

int aipu_free_dma_buf(struct aipu_memory_manager *mm, int fd)
{
	int ret = 0;
	struct aipu_buf_desc buf;
	struct dma_buf *dmabuf = NULL;
	struct aipu_dma_buf_priv *priv = NULL;

	if (!mm || fd <= 0)
		return -EINVAL;

	dmabuf = dma_buf_get(fd);
	if (!dmabuf)
		return -EINVAL;

	priv = (struct aipu_dma_buf_priv *)dmabuf->priv;

	buf.pa = priv->dev_pa;
	buf.dev_offset = priv->dev_pa;
	buf.bytes = priv->bytes;
	buf.region = AIPU_BUF_REGION_DEFAULT;
	buf.asid = AIPU_BUF_ASID_0;
	ret = aipu_mm_free(mm, &buf, NULL);

	if (priv->sgt) {
		devm_kfree(mm->dev, priv->sgt);
		priv->sgt = NULL;
	}

	devm_kfree(mm->dev, priv);
	dma_buf_put(dmabuf);
	return ret;
}

int aipu_get_dma_buf_info(struct aipu_dma_buf *dmabuf_info)
{
	struct dma_buf *dmabuf = NULL;
	struct aipu_dma_buf_priv *priv = NULL;

	if (!dmabuf_info || dmabuf_info->fd <= 0)
		return -EINVAL;

	dmabuf = dma_buf_get(dmabuf_info->fd);
	if (!dmabuf)
		return -EINVAL;

	priv = (struct aipu_dma_buf_priv *)dmabuf->priv;
	dmabuf_info->pa = priv->dev_pa;
	dmabuf_info->bytes = priv->bytes;

	dma_buf_put(dmabuf);
	return 0;
}

#if KERNEL_VERSION(5, 4, 0) < LINUX_VERSION_CODE
MODULE_IMPORT_NS(DMA_BUF);
#endif