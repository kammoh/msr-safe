/* ----------------------------------------------------------------------- *
 *
 *   Copyright 2000-2008 H. Peter Anvin - All Rights Reserved
 *   Copyright 2009 Intel Corporation; author: H. Peter Anvin
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version; incorporated herein by reference.
 *
 * ----------------------------------------------------------------------- */

/*
 * x86 MSR access device
 *
 * This device is accessed by lseek() to the appropriate register number
 * and then read/write in chunks of 8 bytes.  A larger size means multiple
 * reads or writes of the same register.
 *
 * This driver uses /dev/cpu/%d/msr where %d is the minor number, and on
 * an SMP box will direct the access to CPU %d.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <asm/cpufeature.h>
#include <asm/msr.h>
#include <linux/cpu.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "msr_batch.h"
#include "msr_whitelist.h"

static struct class *msr_class;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
static enum cpuhp_state cpuhp_msr_state;
#endif
static int majordev;

static loff_t msr_seek(struct file *file, loff_t offset, int orig)
{
    loff_t ret;
    struct inode *inode = file->f_mapping->host;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    inode_lock(inode);
#else
    mutex_lock(&inode->i_mutex);
#endif
    switch (orig)
    {
        case SEEK_SET:
            file->f_pos = offset;
            ret = file->f_pos;
            break;
        case SEEK_CUR:
            file->f_pos += offset;
            ret = file->f_pos;
            break;
        default:
            ret = -EINVAL;
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    inode_unlock(inode);
#else
    mutex_unlock(&inode->i_mutex);
#endif
    return ret;
}

static ssize_t msr_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    u32 __user *tmp = (u32 __user *) buf;
    u32 data[2];
    u32 reg = *ppos;
    int cpu = iminor(file->f_path.dentry->d_inode);
    int err = 0;
    ssize_t bytes = 0;

    if (count % 8)
    {
        return -EINVAL; /* Invalid chunk size */
    }
#if 0
    if (!capable(CAP_SYS_RAWIO) && !msr_whitelist_maskexists(reg))
    {
        return -EACCES;
    }
#endif
    for (; count; count -= 8)
    {
        err = rdmsr_safe_on_cpu(cpu, reg, &data[0], &data[1]);
        if (err)
        {
            break;
        }
        if (copy_to_user(tmp, &data, 8))
        {
            err = -EFAULT;
            break;
        }
        tmp += 2;
        bytes += 8;
    }

    return bytes ? bytes : err;
}

static ssize_t msr_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    const u32 __user *tmp = (const u32 __user *)buf;
    u32 curdata[2];
    u32 data[2];
    u32 reg = *ppos;
    u64 mask;
    int cpu = iminor(file->f_path.dentry->d_inode);
    int err = 0;
    ssize_t bytes = 0;

    if (count % 8)
    {
        return -EINVAL; // Invalid chunk size
    }

#if 0    
    mask = capable(CAP_SYS_RAWIO) ? 0xffffffffffffffff : msr_whitelist_writemask(reg);

    if (!capable(CAP_SYS_RAWIO) && mask == 0)
    {
        return -EACCES;
    }
#endif
    for (; count; count -= 8)
    {
        if (copy_from_user(&data, tmp, 8))
        {
            err = -EFAULT;
            break;
        }
#if 0
        if (mask != 0xffffffffffffffff)
        {
            err = rdmsr_safe_on_cpu(cpu, reg, &curdata[0], &curdata[1]);
            if (err)
            {
                break;
            }

            *(u64 *)&curdata[0] &= ~mask;
            *(u64 *)&data[0] &= mask;
            *(u64 *)&data[0] |= *(u64 *)&curdata[0];
        }
#endif
        err = wrmsr_safe_on_cpu(cpu, reg, data[0], data[1]);
        if (err)
        {
            break;
        }
        tmp += 2;
        bytes += 8;
    }

    return bytes ? bytes : err;
}

static long msr_ioctl(struct file *file, unsigned int ioc, unsigned long arg)
{
    u32 __user *uregs = (u32 __user *)arg;
    u32 regs[8];
    int cpu = iminor(file->f_path.dentry->d_inode);
    int err;
#if 0
    if (!capable(CAP_SYS_RAWIO))
    {
        return -EACCES;
    }
#endif
    switch (ioc)
    {
        case X86_IOC_RDMSR_REGS:
            if (!(file->f_mode & FMODE_READ))
            {
                err = -EBADF;
                break;
            }
            if (copy_from_user(&regs, uregs, sizeof(regs)))
            {
                err = -EFAULT;
                break;
            }
            err = rdmsr_safe_regs_on_cpu(cpu, regs);
            if (err)
            {
                break;
            }
            if (copy_to_user(uregs, &regs, sizeof(regs)))
            {
                err = -EFAULT;
            }
            break;
        case X86_IOC_WRMSR_REGS:
            if (!(file->f_mode & FMODE_WRITE))
            {
                err = -EBADF;
                break;
            }
            if (copy_from_user(&regs, uregs, sizeof(regs)))
            {
                err = -EFAULT;
                break;
            }
            err = wrmsr_safe_regs_on_cpu(cpu, regs);
            if (err)
            {
                break;
            }
            if (copy_to_user(uregs, &regs, sizeof(regs)))
            {
                err = -EFAULT;
            }
            break;
        default:
            err = -ENOTTY;
            break;
    }

    return err;
}

static int msr_open(struct inode *inode, struct file *file)
{
    unsigned int cpu = iminor(file->f_path.dentry->d_inode);
    struct cpuinfo_x86 *c;

    if (cpu >= nr_cpu_ids || !cpu_online(cpu))
    {
        return -ENXIO;  // No such CPU
    }

    c = &cpu_data(cpu);
    if (!cpu_has(c, X86_FEATURE_MSR))
    {
        return -EIO; // MSR not supported
    }

    return 0;
}

static int msr_close(struct inode *inode, struct file *file)
{
    return 0;
}

/* File operations we support */
static const struct file_operations msr_fops =
{
    .owner = THIS_MODULE,
    .llseek = msr_seek,
    .read = msr_read,
    .write = msr_write,
    .open = msr_open,
    .unlocked_ioctl = msr_ioctl,
    .compat_ioctl = msr_ioctl,
    .release = msr_close
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
static int msr_device_create(int cpu)
#else
static int msr_device_create(unsigned int cpu)
#endif
{
    struct device *dev;

    dev = device_create(msr_class, NULL, MKDEV(majordev, cpu), NULL, "pulsar_msr%d", cpu);
    return IS_ERR(dev) ? PTR_ERR(dev) : 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
static void msr_device_destroy(int cpu)
{
    device_destroy(msr_class, MKDEV(majordev, cpu));
}
#else
static int msr_device_destroy(unsigned int cpu)
{
    device_destroy(msr_class, MKDEV(majordev, cpu));
    return 0;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
static int msr_class_cpu_callback(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
    unsigned int cpu = (unsigned long)hcpu;
    int err = 0;

    switch (action)
    {
        case CPU_UP_PREPARE:
            err = msr_device_create(cpu);
            break;
        case CPU_UP_CANCELED:
        case CPU_UP_CANCELED_FROZEN:
        case CPU_DEAD:
            msr_device_destroy(cpu);
            break;
    }
    return notifier_from_errno(err);
}

static struct notifier_block __refdata msr_class_cpu_notifier =
{
    .notifier_call = msr_class_cpu_callback,
};
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,39)
static char *msr_devnode(struct device *dev, mode_t *mode)
#else
static char *msr_devnode(struct device *dev, umode_t *mode)
#endif
{
    return kasprintf(GFP_KERNEL, "cpu/%u/pulsar_msr", MINOR(dev->devt));
}

static void printc4(void) {
    typedef long unsigned int uint64_t;
    uint64_t output;
    // Read back CR4 to check the bit.
    __asm__("\t mov %%cr4,%0" : "=r"(output));
    printk(KERN_INFO "%lu", output);
}

static void setc4b8(void * info) {
    // Set CR4, Bit 8 (9th bit from the right)  to enable
__asm__("push   %rax\n\t"
                "mov    %cr4,%rax;\n\t"
                "or     $(1 << 8),%rax;\n\t"
                "mov    %rax,%cr4;\n\t"
                "wbinvd\n\t"
                "pop    %rax"
    );

    // Check which CPU we are on:
    printk(KERN_INFO "Ran on Processor %d", smp_processor_id());
    printc4();
}

static void clearc4b8(void * info) {
    printc4();
__asm__("push   %rax\n\t"
        "push   %rbx\n\t"
                "mov    %cr4,%rax;\n\t"
                "mov  $(1 << 8), %rbx\n\t"
                "not  %rbx\n\t"
                "and   %rbx, %rax;\n\t"
                "mov    %rax,%cr4;\n\t"
                "wbinvd\n\t"
                "pop    %rbx\n\t"
                "pop    %rax\n\t"
    );
    printk(KERN_INFO "Ran on Processor %d", smp_processor_id());
}

static int __init msr_init(void)
{
    int err;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
    int i;
#endif
    on_each_cpu(setc4b8, NULL, 0);

    err = msrbatch_init();
    if (err != 0)
    {
        pr_debug("failed to initialize msrbatch\n");
        goto out;
    }

#if 0
    err = msr_whitelist_init();
    if (err != 0)
    {
        pr_debug("failed to initialize whitelist for msr\n");
        goto out_batch;
    }
#endif

    majordev = __register_chrdev(0, 0, num_possible_cpus(), "cpu/pulsar_msr", &msr_fops);
    if (majordev < 0)
    {
        pr_debug("unable to get major %d for pulsar_msr\n", majordev);
        err = -EBUSY;
        goto out_wlist;
    }
    msr_class = class_create(THIS_MODULE, "pulsar_msr");
    if (IS_ERR(msr_class))
    {
        err = PTR_ERR(msr_class);
        goto out_chrdev;
    }
    msr_class->devnode = msr_devnode;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
    i = 0;
    for_each_online_cpu(i)
    err = msr_device_create(i);
    if (err != 0)
    {
        goto out_class;
    }
    register_hotcpu_notifier(&msr_class_cpu_notifier);
#else
    err = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "x86/msr:online", msr_device_create, msr_device_destroy);
    if (err < 0)
    {
        goto out_class;
    }
    cpuhp_msr_state = err;
#endif
    return 0;

out_class:
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
    i = 0;
    for_each_online_cpu(i)
    msr_device_destroy(i);
#endif
    class_destroy(msr_class);
out_chrdev:
    __unregister_chrdev(majordev, 0, num_possible_cpus(), "cpu/pulsar_msr");
out_wlist:
#if 0
    msr_whitelist_cleanup();
#endif
out_batch:
    msrbatch_cleanup();
out:
    return err;
}
module_init(msr_init);

static void __exit msr_exit(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
    int cpu = 0;

    for_each_online_cpu(cpu)
    msr_device_destroy(cpu);
#else
    cpuhp_remove_state(cpuhp_msr_state);
#endif
    class_destroy(msr_class);
    __unregister_chrdev(majordev, 0, num_possible_cpus(), "cpu/pulsar_msr");

    on_each_cpu(clearc4b8, NULL, 0);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
    unregister_hotcpu_notifier(&msr_class_cpu_notifier);
#endif
#if 0
    msr_whitelist_cleanup();
#endif
    msrbatch_cleanup();
}

module_exit(msr_exit)

MODULE_AUTHOR("H. Peter Anvin <hpa@zytor.com>, modified for Pulsar by Kamyar<kammoh@gmail.com>");
MODULE_DESCRIPTION("x86 generic MSR driver -- Modified for Pulsar-- ");
MODULE_VERSION("1.2");
MODULE_LICENSE("GPL");
