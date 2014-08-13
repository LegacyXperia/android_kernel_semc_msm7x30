/* drivers/android/pmem_wrapper.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2012, The Linux Foundation. All rights reserved.
 * Copyright (C) 2014  Rudolf Tammekivi <rtammekivi@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/android_pmem.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/msm_ion.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define PMEM_MAX_DEVICES (10)

struct pmem_data {
	struct ion_client *client;
	unsigned int heap_id;
	struct miscdevice dev;
};

struct allocation_data {
	struct ion_client *client;
	struct ion_handle *handle;
	unsigned int heap_id;
	struct vm_area_struct *vma; /* NULL for indirect allocations. */

	ion_phys_addr_t addr;
	size_t len;
	void *kvaddr;
};

static struct pmem_data pmem[PMEM_MAX_DEVICES];

static int get_id(struct file *file)
{
	return MINOR(file->f_dentry->d_inode->i_rdev);
}

/* Detect whether the file is opened from a PMEM driver. */
static int is_pmem_file(struct file *file)
{
	int id;

	if (unlikely(!file || !file->f_dentry || !file->f_dentry->d_inode))
		return 0;

	id = get_id(file);
	return (unlikely(id >= PMEM_MAX_DEVICES ||
		file->f_dentry->d_inode->i_rdev !=
		     MKDEV(MISC_MAJOR, pmem[id].dev.minor))) ? 0 : 1;
}

static int pmem_set_adata(struct allocation_data *adata)
{
	int ret;

	if (!adata) {
		pr_err("%s: No allocation data\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(adata->client) || IS_ERR_OR_NULL(adata->handle)) {
		pr_err("%s: Invalid allocation data\n", __func__);
		return -EPERM;
	}

	ret = ion_phys(adata->client, adata->handle, &adata->addr, &adata->len);
	if (ret) {
		pr_err("%s: Failed to ion_phys ret=%d\n", __func__, ret);
		return ret;
	}

	adata->kvaddr = ion_map_kernel(adata->client, adata->handle);
	if (IS_ERR_OR_NULL(adata->kvaddr)) {
		ret = PTR_ERR(adata->kvaddr);
		adata->kvaddr = NULL;
		pr_err("%s: Failed to ion_map_kernel ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int pmem_remove_adata(struct allocation_data *adata)
{
	if (!adata) {
		pr_err("%s: No allocation data\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(adata->client) || IS_ERR_OR_NULL(adata->handle)) {
		pr_err("%s: Invalid allocation data\n", __func__);
		return -EPERM;
	}

	ion_unmap_kernel(adata->client, adata->handle);
	adata->kvaddr = NULL;

	adata->addr = 0;
	adata->len = 0;

	return 0;
}

static int pmem_allocate(
	struct allocation_data *adata, size_t size, size_t align)
{
	int ret;

	if (!adata) {
		pr_err("%s: No allocation data\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	if (adata->handle) {
		pr_err("%s: Already allocated\n", __func__);
		ret = -EPERM;
		goto err;
	}

	if (IS_ERR_OR_NULL(adata->client) || !adata->heap_id) {
		pr_err("%s: Invalid allocation data\n", __func__);
		ret = -EPERM;
		goto err;
	}

	adata->handle = ion_alloc(adata->client, size, align,
		ION_HEAP(adata->heap_id), 0);
	if (IS_ERR_OR_NULL(adata->handle)) {
		ret = PTR_ERR(adata->handle);
		adata->handle = NULL;
		pr_err("%s: Failed to ion_alloc ret=%d\n", __func__, ret);
		goto err;
	}

	ret = pmem_set_adata(adata);
	if (ret) {
		pr_err("%s: Failed to set allocation data ret=%d\n",
			__func__, ret);
		goto err_free;
	}

	return 0;
err_free:
	ion_free(adata->client, adata->handle);
	adata->handle = NULL;
err:
	return ret;
}

static int pmem_free(struct allocation_data *adata)
{
	int ret;

	if (!adata) {
		pr_err("%s: No allocation data\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(adata->client) || IS_ERR_OR_NULL(adata->handle)) {
		pr_err("%s: Invalid allocation data\n", __func__);
		return -EPERM;
	}

	ret = pmem_remove_adata(adata);
	if (ret) {
		pr_err("%s: Failed to remove allocation data ret=%d\n",
			__func__, ret);
		return ret;
	}

	ion_free(adata->client, adata->handle);
	adata->handle = NULL;

	return 0;
}

int get_pmem_file(unsigned int fd, unsigned long *start, unsigned long *vstart,
		  unsigned long *len, struct file **filep)
{
	int ret;
	struct file *file = fget(fd);
	struct allocation_data *adata = NULL;
	bool pmem_file = is_pmem_file(file);

	if (pmem_file) {
		/* Get file private data, which is stored in pmem_open. */
		adata = file->private_data;
	} else {
		/* Close the file, not using it anymore. */
		fput(file);

		file = kzalloc(sizeof(*file), GFP_KERNEL);
		/* Not PMEM fd. Assume the fd is directly from ION. */
		adata = kzalloc(sizeof(*adata), GFP_KERNEL);
		/* Get PMEM client 0 ION client. */
		adata->client = pmem[0].client;
		if (IS_ERR_OR_NULL(adata->client)) {
			ret = PTR_ERR(adata->client);
			pr_err("%s: Invalid ion client ret=%d\n",
				__func__, ret);
			kfree(adata);
			kfree(file);
			return ret;
		}
		adata->handle = ion_import_dma_buf(adata->client, fd);
		if (IS_ERR_OR_NULL(adata->handle)) {
			ret = PTR_ERR(adata->handle);
			pr_err("%s: Failed to ion_import_dma_buf ret=%d\n",
				__func__, ret);
			kfree(adata);
			kfree(file);
			return ret;
		}
		adata->vma = NULL;

		ret = pmem_set_adata(adata);
		if (ret) {
			pr_err("%s: Failed to set allocation data ret=%d\n",
				__func__, ret);
			ion_free(adata->client, adata->handle);
			kfree(adata);
			kfree(file);
			return ret;
		}

		file->private_data = adata;
	}

	*start = adata->addr;
	*vstart = (unsigned long)adata->kvaddr;
	*len = adata->len;

	*filep = file;

	return 0;
}
EXPORT_SYMBOL(get_pmem_file);

int get_pmem_fd(int fd, unsigned long *start, unsigned long *len)
{
	unsigned long vstart;
	return get_pmem_file(fd, start, &vstart, len, NULL);
}
EXPORT_SYMBOL(get_pmem_fd);

int get_pmem_user_addr(struct file *file, unsigned long *start,
		       unsigned long *len)
{
	int ret = -EINVAL;

	if (is_pmem_file(file)) {
		struct allocation_data *adata = file->private_data;
		if (adata->handle && adata->vma) {
			*start = adata->vma->vm_start;
			*len = adata->vma->vm_end - adata->vma->vm_start;
			ret = 0;
		}
	}
	return ret;
}
EXPORT_SYMBOL(get_pmem_user_addr);

void put_pmem_file(struct file *file)
{
	struct allocation_data *adata = file->private_data;
	bool pmem_file = is_pmem_file(file);

	if (adata->client && adata->handle) {
		if (!pmem_file) {
			pmem_free(adata);
			kfree(adata);
		}
	}

	if (pmem_file)
		fput(file);
	else
		kfree(file);
}
EXPORT_SYMBOL(put_pmem_file);

void put_pmem_fd(int fd)
{
	int put_needed;
	struct file *file = fget_light(fd, &put_needed);

	if (file) {
		put_pmem_file(file);
		fput_light(file, put_needed);
	}
}
EXPORT_SYMBOL(put_pmem_fd);

void flush_pmem_fd(int fd, unsigned long offset, unsigned long len)
{
	int fput_needed;
	struct file *file = fget_light(fd, &fput_needed);

	if (file) {
		flush_pmem_file(file, offset, len);
		fput_light(file, fput_needed);
	}
}
EXPORT_SYMBOL(flush_pmem_fd);

void flush_pmem_file(struct file *file, unsigned long offset, unsigned long len)
{
	struct allocation_data *adata = file->private_data;
	struct pmem_addr addr;

	addr.vaddr = (unsigned long)adata->kvaddr;
	addr.offset = offset;
	addr.length = len;
	pmem_cache_maint(file, PMEM_CLEAN_INV_CACHES, &addr);
}
EXPORT_SYMBOL(flush_pmem_file);

int pmem_cache_maint(struct file *file, unsigned int cmd,
		struct pmem_addr *pmem_addr)
{
	int ret = -EINVAL;
	struct allocation_data *adata = file->private_data;

	if (IS_ERR_OR_NULL(adata->handle))
		return -EINVAL;

	switch (cmd) {
	case PMEM_CLEAN_INV_CACHES:
		ret = msm_ion_do_cache_op(adata->client, adata->handle,
			&pmem_addr->vaddr, pmem_addr->length,
			ION_IOC_CLEAN_INV_CACHES);
		break;
	case PMEM_CLEAN_CACHES:
		ret = msm_ion_do_cache_op(adata->client, adata->handle,
			&pmem_addr->vaddr, pmem_addr->length,
			ION_IOC_CLEAN_CACHES);
		break;
	case PMEM_INV_CACHES:
		ret = msm_ion_do_cache_op(adata->client, adata->handle,
			&pmem_addr->vaddr, pmem_addr->length,
			ION_IOC_INV_CACHES);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, cmd);
		break;
	};
	return ret;
}
EXPORT_SYMBOL(pmem_cache_maint);

static long pmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = -EINVAL;
	struct allocation_data *adata = file->private_data;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case PMEM_GET_SIZE:
	case PMEM_GET_PHYS: {
		struct pmem_region region;

		if (IS_ERR_OR_NULL(adata->handle))
			return -ENOMEM;

		region.offset = adata->addr;
		region.len = adata->len;

		if (copy_to_user(argp, &region, sizeof(region)))
			return -EFAULT;
		ret = 0;
		break;
	}
	case PMEM_MAP:
		pr_debug("%s: Unsupported ioctl PMEM_MAP\n", __func__);
		break;
	case PMEM_UNMAP:
		pr_debug("%s: Unsupported ioctl PMEM_UNMAP\n", __func__);
		break;
	case PMEM_ALLOCATE:
		ret = pmem_allocate(adata, arg, SZ_4K);
		if (ret) {
			pr_err("%s: Failed to pmem_allocate ret=%d\n",
				__func__, ret);
			return ret;
		}
		break;
	case PMEM_CONNECT:
		pr_debug("%s: Unsupported ioctl PMEM_CONNECT\n", __func__);
		break;
	case PMEM_GET_TOTAL_SIZE:
		pr_debug("%s: Unsupported ioctl PMEM_GET_TOTAL_SIZE\n", __func__);
		break;
	case PMEM_CLEAN_INV_CACHES:
	case PMEM_CLEAN_CACHES:
	case PMEM_INV_CACHES: {
		struct pmem_addr pmem_addr;
		if (copy_from_user(&pmem_addr, argp, sizeof(struct pmem_addr)))
			return -EFAULT;
		ret = pmem_cache_maint(file, cmd, &pmem_addr);
		break;
	}
	case PMEM_GET_FREE_SPACE:
		pr_debug("%s: Unsupported ioctl PMEM_GET_FREE_SPACE\n", __func__);
		break;
	case PMEM_ALLOCATE_ALIGNED: {
		struct pmem_allocation alloc;
		if (copy_from_user(&alloc, argp,
			sizeof(struct pmem_allocation)))
			return -EFAULT;
		ret = pmem_allocate(adata, alloc.size, alloc.align);
		if (ret) {
			pr_err("%s: Failed to pmem_allocate ret=%d\n",
				__func__, ret);
			return ret;
		}
		break;
	}
	default:
		pr_err("%s: Unsupported ioctl %d\n", __func__, cmd);
		break;
	};

	return ret;
}

static int pmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret;
	struct allocation_data *adata = file->private_data;
	unsigned long vma_size = vma->vm_end - vma->vm_start;
	bool already_allocated = (adata->handle != NULL);

	if (!already_allocated) {
		ret = pmem_allocate(adata, vma_size, SZ_4K);
		if (ret) {
			pr_err("%s: Failed to pmem_allocate ret=%d\n",
				__func__, ret);
			goto err;
		}
	}

	/* Set physical address link with VMA. */
	vma->vm_pgoff = adata->addr >> PAGE_SHIFT;

	/* MAP physical address to userspace. */
	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, vma_size,
		vma->vm_page_prot);
	if (ret) {
		pr_err("%s: Failed to remap_pfn_range ret=%d\n", __func__, ret);
		goto err_free;
	}

	/* Set vma for future mappings. */
	adata->vma = vma;

	pr_debug("%s: Allocated & mmapped p:0x%lx v:0x%lx\n",
		__func__, adata->addr, vma->vm_start);

	return 0;
err_free:
	if (!already_allocated) {
		pmem_free(adata);
	}
err:
	return ret;
}

static int pmem_open(struct inode *inode, struct file *file)
{
	int id = get_id(file);
	struct allocation_data *adata;

	adata = kzalloc(sizeof(*adata), GFP_KERNEL);
	if (!adata)
		return -ENOMEM;

	adata->client = pmem[id].client;
	adata->heap_id = pmem[id].heap_id;

	file->private_data = adata;

	return 0;
}

static int pmem_release(struct inode *inode, struct file *file)
{
	struct allocation_data *adata = file->private_data;

	pmem_free(adata);

	file->private_data = NULL;
	kfree(adata);

	return 0;
}

static const struct file_operations pmem_fops = {
	.unlocked_ioctl	= pmem_ioctl,
	.mmap		= pmem_mmap,
	.open		= pmem_open,
	.release	= pmem_release,
};

int pmem_setup(struct android_pmem_platform_data *pdata,
	long (*ioctl)(struct file *, unsigned int, unsigned long),
	int (*release)(struct inode *, struct file *))
{
	int ret;

	static int id = 0;

	if (!pdata->ion_heap_id) {
		pr_err("No ion heap mask\n");
		ret = -EINVAL;
		goto err;
	}
	pmem[id].heap_id = pdata->ion_heap_id;
	pmem[id].client = msm_ion_client_create(-1, pdata->name);
	if (IS_ERR_OR_NULL(pmem[id].client)) {
		ret = PTR_ERR(pmem[id].client);
		pmem[id].client = NULL;
		pr_err("Failed to msm_ion_client_create ret=%d\n", ret);
		goto err;
	}

	/* Dynamic devfs node. */
	pmem[id].dev.name = pdata->name;
	pmem[id].dev.minor = id;
	pmem[id].dev.fops = &pmem_fops;

	ret = misc_register(&pmem[id].dev);
	if (ret) {
		pr_err("Failed to misc_register ret=%d\n", ret);
		goto err_destroy_client;
	}

	id++;

	return 0;
err_destroy_client:
	ion_client_destroy(pmem[id].client);
	pmem[id].client = NULL;
err:
	return ret;
}
EXPORT_SYMBOL(pmem_setup);

static int pmem_probe(struct platform_device *pdev)
{
	struct android_pmem_platform_data *pdata;

	if (!pdev || !pdev->dev.platform_data) {
		pr_err("No pdev/platform_data\n");
		return -EINVAL;
	}
	pdata = pdev->dev.platform_data;

	return pmem_setup(pdata, NULL, NULL);
}

static int pmem_remove(struct platform_device *pdev)
{
	int id = pdev->id;

	misc_deregister(&pmem[id].dev);

	ion_client_destroy(pmem[id].client);
	pmem[id].client = NULL;
	return 0;
}

static struct platform_driver pmem_driver = {
	.probe	= pmem_probe,
	.remove	= pmem_remove,
	.driver	= {
		.name = "android_pmem",
	}
};


static int __init pmem_init(void)
{
	return platform_driver_register(&pmem_driver);
}

static void __exit pmem_exit(void)
{
	platform_driver_unregister(&pmem_driver);
}

module_init(pmem_init);
module_exit(pmem_exit);
