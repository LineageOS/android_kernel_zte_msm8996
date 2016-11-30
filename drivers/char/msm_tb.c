/*
 *  linux/drivers/char/msm_tb.c
 *
 *
 *  Used for qualcomm trace tool, to map the reserved trace buffer to user space.
 */

#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/raw.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/crash_dump.h>
#include <linux/backing-dev.h>
#include <linux/bootmem.h>
#include <linux/splice.h>
#include <linux/pfn.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/aio.h>
#include <linux/of_address.h>

#include <asm/uaccess.h>

#ifdef CONFIG_IA64
# include <linux/efi.h>
#endif


#define SDLOG_MEM_PHYS_ADDR_PROP "qcom,msm-imem-sdlog_mem_phys_addr"
#define SDLOG_MEM_SIZE_PROP "qcom,msm-imem-sdlog_mem_size"


/*
 *  the definitions for major device numbers, make sure not conflict with system
 *  pre-defined, refer Major.h 
 */
#define MSM_TB_MAJOR   60

/*
 *  trace buffer phy addr, must confirm with boot for the correct addr for every platform
 */

u64 sdlog_mem_phys_addr = 0;
u32 sdlog_mem_size = 0;

static void *sdlog_mem_phys_imem_addr = NULL;
static void *sdlog_mem_size_imem_addr = NULL;


extern int sdlog_memory_get_size(void);
extern unsigned int sdlog_memory_get_addr(void);


static int is_f3mem_range(unsigned long pfn, unsigned long size)
{
	u64 from = ((u64)pfn) << PAGE_SHIFT;
	u64 to = from + size;
	pr_info("is_f3mem_range: from 0x%llx, to 0x%llx\n", from, to);
	if( (from < sdlog_mem_phys_addr) || (to > (sdlog_mem_phys_addr + sdlog_mem_size)) )
	{
		return 0;
	}
	
	pr_info("is_f3mem_range: belong to f3mem\n");
	return 1;
}

static inline int range_is_allowed(unsigned long pfn, unsigned long size)
{
	if (is_f3mem_range(pfn, size))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*
 * This funcion reads the *physical* memory. The f_pos points directly to the
 * memory location.
 */
static ssize_t msm_tb_read_mem(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
    printk("msm_tb_read_mem - not supported \n");
    return -EFAULT;
}

static ssize_t msm_tb_write_mem(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
    printk("msm_tb_read_mem - not supported \n");
    return -EFAULT;
}


int __weak phys_mem_access_prot_allowed(struct file *file,
    unsigned long pfn, unsigned long size, pgprot_t *vma_prot)
{
    return 1;
}


static inline int private_mapping_ok(struct vm_area_struct *vma)
{
    return 1;
}

static const struct vm_operations_struct msm_tb_mmap_mem_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
    .access = generic_access_phys
#endif
};

static int msm_tb_mmap_mem(struct file *file, struct vm_area_struct *vma)
{
    size_t size = vma->vm_end - vma->vm_start;

    printk("msm_tb_mmap_mem-vm_start:0x%lx,vm_end:0x%lx,pgoff:0x%lx \n",
           vma->vm_start, vma->vm_end, vma->vm_pgoff);

    if (!valid_mmap_phys_addr_range(vma->vm_pgoff, size))
    {
        printk("valid_mmap_phys_addr_range - failed \n");
        return -EINVAL;
    }

    if (!private_mapping_ok(vma))
    {
        printk("private_mapping_ok - failed \n");
        return -ENOSYS;
    }

    if (!range_is_allowed(vma->vm_pgoff, size))
    {
        printk("range_is_allowed - failed \n");
        return -EPERM;
    }

    if (!phys_mem_access_prot_allowed(file, vma->vm_pgoff, size,
                                      &vma->vm_page_prot))
    {
        printk("phys_mem_access_prot_allowed - failed \n");
        return -EINVAL;
    }

    vma->vm_page_prot = phys_mem_access_prot(file, vma->vm_pgoff,
                                             size,
                                             vma->vm_page_prot);

    vma->vm_ops = &msm_tb_mmap_mem_ops;

    /* Remap-pfn-range will mark the range VM_IO */
    if (remap_pfn_range(vma,
                        vma->vm_start,
                        vma->vm_pgoff,
                        size,
                        vma->vm_page_prot)) {

        printk("msm_tb_mmap_mem - remap_pfn_range failed \n");
        return -EAGAIN;
    }

    printk("msm_tb_mmap_mem - remap_pfn_range succeed \n");
    return 0;
}

/*
 * The memory devices use the full 32/64 bits of the offset, and so we cannot
 * check against negative addresses: they are ok. The return value is weird,
 * though, in that case (0).
 *
 * also note that seeking relative to the "end of file" isn't supported:
 * it has no meaning, so it returns -EINVAL.
 */
static loff_t msm_tb_memory_lseek(struct file *file, loff_t offset, int orig)
{
    printk("msm_tb_memory_lseek - not supported \n");
    return -EINVAL;
}


static int msm_tb_open_port(struct inode *inode, struct file *filp)
{
    printk("msm_tb_open_port \n");
    return  0;
}

static const struct file_operations msm_tb_fops = {
    .llseek		= msm_tb_memory_lseek,
    .read		= msm_tb_read_mem,
    .write		= msm_tb_write_mem,
    .mmap		= msm_tb_mmap_mem,
    .open		= msm_tb_open_port,
};

static struct class *msm_tb_class;

static int __init msm_tb_dev_init(void)
{
    int minor = 1;
    struct device_node *np;
    
    //Add by ruijiagui, map imem sdlog mem address
    np = of_find_compatible_node(NULL, NULL, SDLOG_MEM_PHYS_ADDR_PROP);
    if (!np) {
      pr_info("unable to find DT imem sdlog mem addr node, use sdlog addr\n");
			sdlog_mem_phys_addr = sdlog_memory_get_addr();
    } else {
      sdlog_mem_phys_imem_addr = of_iomap(np, 0);
      if (sdlog_mem_phys_imem_addr)
      {
        sdlog_mem_phys_addr = *((u32*)sdlog_mem_phys_imem_addr);
      }
      else
      {
        pr_err("unable to map imem sdlog mem addr offset\n");
      }
    }

      //Add by ruijiagui, map imem sdlog mem address
    np = of_find_compatible_node(NULL, NULL, SDLOG_MEM_SIZE_PROP);
    if (!np) {
      pr_info("unable to find DT imem sdlog mem size node, use sdlog size\n");
			sdlog_mem_size = sdlog_memory_get_size();
    } else {
      sdlog_mem_size_imem_addr = of_iomap(np, 0);
      if (sdlog_mem_size_imem_addr)
      {
        sdlog_mem_size = *((u32*)sdlog_mem_size_imem_addr);
      }
      else
      {
        pr_err("unable to map imem sdlog mem size offset\n");
      }
    }

    if (register_chrdev(MSM_TB_MAJOR, "msm_tb", &msm_tb_fops))
        printk("unable to get major %d for memory devs\n", MSM_TB_MAJOR);

    printk("msm_tb_dev_init \n");

    msm_tb_class = class_create(THIS_MODULE, "msm_tb");
    if (IS_ERR(msm_tb_class))
        return PTR_ERR(msm_tb_class);

    device_create(msm_tb_class, NULL, MKDEV(MSM_TB_MAJOR, minor), NULL, "msm_tb");


    printk("msm_tb_dev_init - succeed\n");
    return 0;
}

fs_initcall(msm_tb_dev_init);

