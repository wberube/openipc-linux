/*
* pm.c- Sigmastar
*
* Copyright (c) [2019~2020] SigmaStar Technology.
*
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License version 2 for more details.
*
*/
#include <linux/suspend.h>
#include <linux/io.h>
#include <asm/suspend.h>
#include <asm/fncpy.h>
#include <asm/cacheflush.h>
#include "ms_platform.h"
#include <linux/delay.h>
#include <asm/secure_cntvoff.h>

#include "voltage_ctrl.h"
#include "registers.h"
#include "vad_STR.h"
#include "mailbox_str.h"

extern void cpu_suspend_abort(void);

#define FIN     printk(KERN_ERR"[%s]+++\n",__FUNCTION__)
#define FOUT    printk(KERN_ERR"[%s]---\n",__FUNCTION__)
#define HERE    printk(KERN_ERR"%s: %d\n",__FILE__,__LINE__)
#define SUSPEND_WAKEUP 0
#define SUSPEND_SLEEP  1
#define STR_PASSWORD   0x5A5A55AA

#define SUSPEND_CODE_SIZE 0x1000
#define RESUME_CODE_SIZE 0x1000
#define STACK_SIZE 0x3000
#define LIB_SIZE 0x19000 //lib size+Working buffer
#define RAM_SIZE 0x3000 //ram section3KB
#define FRAME_SIZE 0x3000
#define INFO_SIZE 0x1000

#define SUSPEND_START 0xA0000000
#define RESUME_START (SUSPEND_START + SUSPEND_CODE_SIZE)
#define STACK_START (RESUME_START + RESUME_CODE_SIZE)
#define LIB_START (STACK_START + STACK_SIZE)

#define TIME_START (0xA0028400)

#define RAM_START (TIME_START + INFO_SIZE)
#define FRAME_START (RAM_START + RAM_SIZE)
#define INFO_START (FRAME_START + FRAME_SIZE)

//#define STACK_GAP 0x100

extern void vad_main(void);

typedef struct {
    char magic[8];
    unsigned int resume_entry;
    unsigned int count;
    unsigned int status;
    unsigned int password;
    unsigned int vadFrameAvg;
    unsigned int vadFrameResult;
    unsigned int vadCpuFreq;
    char vadLibVersion[32];
} suspend_keep_info;

unsigned int resume_pbase = 0;
extern void sram_suspend_imi(void) __attribute__((aligned(16)));
void (*mstar_resume_imi_fn)(void);
void(sram_resume_imi)(void) __attribute__((aligned(8)));
void __iomem *suspend_imi_vbase;
void __iomem *resume_imi_vbase;
void __iomem *resume_imistack_vbase;
void __iomem *lib_vbase;
void __iomem *ram_vbase;
void __iomem *info_vbase;
void __iomem *frame_vbase;
void __iomem *timestemp_vbase;

static void (*mstar_suspend_imi_fn)(void);
static suspend_keep_info *pStr_info;
int suspend_status = SUSPEND_WAKEUP;

static u32 CurTask_SP = 0;
static u32 Reglr;

void sram_resume_imi(void)
{
    *(unsigned short volatile *)0xFD200800 = WAKEUP_SRAM_7;

    asm volatile("mov lr, %0" : : "r"(Reglr));
    asm volatile("mov sp, %0" : : "r"(CurTask_SP));
    asm volatile("ldmia.w sp!, {r3-r9, lr}");

    asm volatile("ldmia   sp!, {r1 - r3}"); //@ pop phys pgd, virt SP, phys resume fn
    asm volatile("teq r0, #0");
    asm volatile("moveq   r0, #1"); //e force non-zero value
    asm volatile("mov sp, r2");
    asm volatile("ldmfd   sp!, {r4 - r11, pc}");
}

#define GET_REG8_ADDR(x, y) (x + (y)*2)
#define GET_REG16_ADDR(x, y) (x + (y)*4)
#define RIU_BASE_ADDR 0xFD000000

#ifdef CONFIG_MS_AIO
extern signed int MHAL_AUDIO_AI_DmaImiEnable(bool bEnable);
extern signed int MHAL_AUDIO_AI_DmaImiInit(unsigned long long nBufAddrOffset, unsigned long nBufSize);
#endif

int sstar_suspend_ready(unsigned long ret)
{
    int resume_fn, stack;

    *(unsigned short volatile *)0xFD200800 = SUSPEND_READY_0;

#ifdef CONFIG_MS_AIO
#if defined(CONFIG_SS_PM_VAD_LOOP_MODE) || defined(CONFIG_SS_PM_VAD_INT_MODE)
    //enable sig_gen
    //*(unsigned short volatile *)0xFD2A0750 = 0xF015;
    MHAL_AUDIO_AI_DmaImiInit(0xA002c000, 0x3000); //12KB
#if defined(CONFIG_SS_PM_VAD_LOOP_MODE)
    MHAL_AUDIO_AI_DmaImiEnable(TRUE);
#endif
#endif
#endif

    *(unsigned short volatile *)0xFD200800 = SUSPEND_READY_1;

    pStr_info = (suspend_keep_info *)(info_vbase);
    memset(pStr_info, 0, sizeof(suspend_keep_info));
    strcpy(pStr_info->magic, "SIG_STR");
    pStr_info->resume_entry = resume_pbase;

    mstar_suspend_imi_fn = fncpy(suspend_imi_vbase, (void *)&sram_suspend_imi, SUSPEND_CODE_SIZE);
    suspend_status = SUSPEND_SLEEP;
    mstar_resume_imi_fn = fncpy(resume_imi_vbase, (void *)&sram_resume_imi, RESUME_CODE_SIZE);

    stack = (u32)resume_imistack_vbase + STACK_SIZE - 0x10;
    resume_fn = (u32)mstar_resume_imi_fn;
    //copy
    memcpy(lib_vbase, vad_str, sizeof(vad_str));

    //resume info
    if (pStr_info) {
        pStr_info->count++;
        pStr_info->status = SUSPEND_SLEEP;
        pStr_info->password = STR_PASSWORD;
    }
    asm volatile("mov %0, sp" : "=r"(CurTask_SP));
    asm volatile("mov %0, lr" : "=r"(Reglr));
    //flush cache to ensure memory is updated before self-refresh
    __cpuc_flush_kern_all();
    //flush L3 cache
    Chip_Flush_MIU_Pipe();

    *(unsigned short volatile *)0xFD200800 = SUSPEND_READY_2;
    *(unsigned short volatile *)0xFD200808 = (u32)resume_fn;
    *(unsigned short volatile *)0xFD200804 = (u32)(resume_fn >> 16);
    *(unsigned short volatile *)0xFD200810 = (u32)stack;
    *(unsigned short volatile *)0xFD20080C = (u32)(stack >> 16);
    *(unsigned short volatile *)0xFD200800 = SUSPEND_READY_3;

    mstar_suspend_imi_fn();
    return 0;
}

static int sstar_suspend_enter(suspend_state_t state)
{
    //FIN;
    switch (state)
    {
        case PM_SUSPEND_MEM:
            printk(KERN_INFO "state = PM_SUSPEND_MEM\n");
            cpu_suspend(0, sstar_suspend_ready);
#ifdef CONFIG_SMP
            secure_cntvoff_init();
#endif
            break;
        default:
            return -EINVAL;
    }

    return 0;
}

static void sstar_suspend_wake(void)
{
    if (pStr_info) {
        pStr_info->status = SUSPEND_WAKEUP;
        pStr_info->password = 0;
    }
}

struct platform_suspend_ops mstar_suspend_ops = {
    .enter = sstar_suspend_enter,
    .wake = sstar_suspend_wake,
    .valid = suspend_valid_only_mem,
};

int __init mstar_pm_init(void)
{
    resume_pbase = virt_to_phys(cpu_resume);
    suspend_imi_vbase = (void __iomem *)IMI_VIRT;
    resume_imi_vbase = (void __iomem *)IMI_VIRT + 0X1000;
    //    suspend_info_vbase =  __arm_ioremap_exec(SUSPENDINFO_ADDRESS, 0x1000, false);
    resume_imistack_vbase = (void __iomem *)IMI_VIRT + 0X2000;
    lib_vbase = (void __iomem *)IMI_VIRT + 0X5000;
    timestemp_vbase = (void __iomem *)IMI_VIRT + 0X28400;
    ram_vbase = (void __iomem *)IMI_VIRT + 0X29000;
    frame_vbase = (void __iomem *)IMI_VIRT + 0X2c000;
    info_vbase = (void __iomem *)IMI_VIRT + 0X2f000;
    memset(suspend_imi_vbase, 0, 0x10000);

    suspend_set_ops(&mstar_suspend_ops);
    printk(KERN_INFO "[%s] resume_pbase=0x%08X\n", __func__,
        (unsigned int)resume_pbase);
    printk(KERN_INFO "[%s] suspend_code_vbase=0x%08X, PA:0x%x size 0x%x\n",
        __func__, (unsigned int)suspend_imi_vbase, SUSPEND_START,
        SUSPEND_CODE_SIZE);
    printk(KERN_INFO "[%s] resume_code_vbase=0x%08X, PA:0x%x size 0x%x\n",
        __func__, (unsigned int)resume_imi_vbase, RESUME_START,
        RESUME_CODE_SIZE);
    printk(KERN_INFO "[%s] stack_vbase=0x%08X, PA:0x%x size 0x%x\n",
        __func__, (unsigned int)resume_imistack_vbase, STACK_START,
        STACK_SIZE);
    printk(KERN_INFO "[%s] lib_vbase=0x%08X, PA:0x%x size 0x%x\n", __func__,
        (unsigned int)lib_vbase, LIB_START, LIB_SIZE);

    printk(KERN_INFO "[%s] timestemp_vbase=0x%08X, PA:0x%x size 0x%x\n",
        __func__, (unsigned int)timestemp_vbase, TIME_START, INFO_SIZE);
    printk(KERN_INFO "[%s] ram_vbase=0x%08X, PA:0x%x size 0x%x\n", __func__,
        (unsigned int)ram_vbase, RAM_START, RAM_SIZE);
    printk(KERN_INFO "[%s] info_vbase=0x%08X, PA:0x%x size 0x%x\n",
        __func__, (unsigned int)info_vbase, INFO_START, INFO_SIZE);
    printk(KERN_INFO "[%s] frame_vbase=0x%08X, PA:0x%x size 0x%x\n",
        __func__, (unsigned int)frame_vbase, FRAME_START, FRAME_SIZE);

    return 0;
}
