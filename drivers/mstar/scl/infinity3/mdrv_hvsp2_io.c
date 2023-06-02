////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2011 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// ("MStar Confidential Information") by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

#include <linux/pfn.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>          /* seems do not need this */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/string.h>

#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include "MsCommon.h"
#include "MsTypes.h"
#include "MsOS.h"
#include "ms_msys.h"
#include "mdrv_hvsp_io_i3_st.h"
#include "mdrv_hvsp_io_i3.h"
#include "mdrv_scl_dbg.h"
#include "mdrv_hvsp.h"
#include "mdrv_multiinst.h"
#include "mdrv_verchk.h"
//-------------------------------------------------------------------------------------------------

#define MDRV_MS_HVSP_DEVICE_COUNT   1
#define MDRV_MS_HVSP_NAME           "mhvsp2"
#define MAX_FILE_HANDLE_SUPPRT      64
#define MDRV_NAME_HVSP              "mhvsp2"
#define MDRV_MAJOR_HVSP             0xea
#define MDRV_MINOR_HVSP             0x02
//-------------------------------------------------------------------------------------------------

#define CMD_PARSING(x)  (x==IOCTL_HVSP_SET_IN_CONFIG         ?  "IOCTL_HVSP_SET_IN_CONFIG" : \
                         x==IOCTL_HVSP_SET_OUT_CONFIG        ?  "IOCTL_HVSP_SET_OUT_CONFIG" : \
                         x==IOCTL_HVSP_SET_SCALING_CONFIG    ?  "IOCTL_HVSP_SET_SCALING_CONFIG" : \
                         x==IOCTL_HVSP_REQ_MEM_CONFIG        ?  "IOCTL_HVSP_REQ_MEM_CONFIG" : \
                         x==IOCTL_HVSP_SET_MISC_CONFIG       ?  "IOCTL_HVSP_SET_MISC_CONFIG" : \
                         x==IOCTL_HVSP_GET_PRIVATE_ID_CONFIG ?  "IOCTL_HVSP_GET_PRIVATE_ID_CONFIG" : \
                                                                "UNKNOWN")

//-------------------------------------------------------------------------------------------------

int mdrv_ms_hvsp2_open(struct inode *inode, struct file *filp);
int mdrv_ms_hvsp2_release(struct inode *inode, struct file *filp);
long mdrv_ms_hvsp2_ioctl(struct file *filp, unsigned int u32Cmd, unsigned long u32Arg);
static int mdrv_ms_hvsp2_probe(struct platform_device *pdev);
static int mdrv_ms_hvsp2_remove(struct platform_device *pdev);
static int mdrv_ms_hvsp2_suspend(struct platform_device *dev, pm_message_t state);
static int mdrv_ms_hvsp2_resume(struct platform_device *dev);
static unsigned int mdrv_ms_hvsp2_poll(struct file *filp, struct poll_table_struct *wait);

//-------------------------------------------------------------------------------------------------

typedef struct
{
    int s32Major;
    int s32Minor;
    int refCnt;
    struct cdev cdev;
    struct file_operations fops;
    ST_MDRV_HVSP_CLK_CONFIG stclk;
	struct device *devicenode;
}ST_DEV_HVSP;

static ST_DEV_HVSP _dev_ms_hvsp2 =
{
    .s32Major = MDRV_MAJOR_HVSP,
    .s32Minor = MDRV_MINOR_HVSP,
    .refCnt = 0,
    .cdev =
    {
        .kobj = {.name= MDRV_NAME_HVSP, },
        .owner = THIS_MODULE,
    },
    .fops =
    {
        .open = mdrv_ms_hvsp2_open,
        .release = mdrv_ms_hvsp2_release,
        .unlocked_ioctl = mdrv_ms_hvsp2_ioctl,
        .poll = mdrv_ms_hvsp2_poll,
    }
};

static struct class * m_hvsp2_class = NULL;
static char * hvsp2_classname = "m_hvsp2_class";


static const struct of_device_id ms_hvsp2_of_match_table[] =
{
    { .compatible = "mstar,hvsp2" },
    {}
};

static struct platform_driver st_ms_hvsp2_driver =
{
	.probe 		= mdrv_ms_hvsp2_probe,
	.remove 	= mdrv_ms_hvsp2_remove,
    .suspend    = mdrv_ms_hvsp2_suspend,
    .resume     = mdrv_ms_hvsp2_resume,
	.driver =
	{
		.name	= MDRV_NAME_HVSP,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(ms_hvsp2_of_match_table),
	},
};

static u64 ms_hvsp2_dma_mask = 0xffffffffUL;

static struct platform_device st_ms_hvsp2_device =
{
    .name = "mhvsp2",
    .id = 0,
    .dev =
    {
        .dma_mask = &ms_hvsp2_dma_mask,
        .coherent_dma_mask = 0xffffffffUL
    }
};
MS_U8 gu8sc2first;
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
// IOCtrl Driver interface functions
//-------------------------------------------------------------------------------------------------
static ssize_t check_osd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	if(NULL!=buf)
	{
        MDrv_HVSP_OsdStore(buf,E_MDRV_HVSP_ID_2);
        return n;
	}

	return 0;
}
static ssize_t check_osd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return MDrv_HVSP_OsdShow(buf);
}
static DEVICE_ATTR(osd,0644, check_osd_show, check_osd_store);
static ssize_t check_SCIQ_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
    MDrv_HVSP_SCIQStore(buf,E_MDRV_HVSP_ID_2);
    return n;
}
static ssize_t check_SCIQ_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return MDrv_HVSP_SCIQShow(buf,E_MDRV_HVSP_ID_2);
}

static DEVICE_ATTR(SCIQ,0644, check_SCIQ_show, check_SCIQ_store);
void _mdrv_ms_hvsp2_io_fill_versionchkstruct
(unsigned int u32StructSize,unsigned int u32VersionSize,unsigned int *pVersion,ST_MDRV_HVSP_VERSIONCHK_CONFIG *stVersion)
{
    stVersion->u32StructSize  = (unsigned int)u32StructSize;
    stVersion->u32VersionSize = (unsigned int)u32VersionSize;
    stVersion->pVersion      = (unsigned int *)pVersion;
}
int _mdrv_ms_hvsp2_io_version_check(ST_MDRV_HVSP_VERSIONCHK_CONFIG *stVersion)
{
    if ( CHK_VERCHK_HEADER(stVersion->pVersion) )
    {
        if( CHK_VERCHK_MAJORVERSION_LESS( stVersion->pVersion, IOCTL_HVSP_VERSION) )
        {

            VERCHK_ERR("[HVSP2] Version(%04x) < %04x!!! \n",
                *(stVersion->pVersion) & VERCHK_VERSION_MASK,
                IOCTL_HVSP_VERSION);

            return -EINVAL;
        }
        else
        {
            if( CHK_VERCHK_SIZE( &stVersion->u32VersionSize, stVersion->u32StructSize) == 0 )
            {
                VERCHK_ERR("[HVSP2] Size(%04x) != %04x!!! \n",
                    stVersion->u32StructSize,
                    stVersion->u32VersionSize);

                return -EINVAL;
            }
            else
            {
                    return VersionCheckSuccess;
            }
        }
    }
    else
    {
        VERCHK_ERR("[HVSP2] No Header !!! \n");
        SCL_ERR( "[HVSP2]   %s  \n", __FUNCTION__);
        return -EFAULT;
    }
}

static int _ms_hvsp2_multiinstSet(EN_MDRV_MULTI_INST_CMD_TYPE enType, void *stCfg ,void *privatedata)
{
    EN_MDRV_MULTI_INST_STATUS_TYPE enMultiInstRet;
    int ret = 0;
    enMultiInstRet = MDrv_MultiInst_Entry_FlashData(E_MDRV_MULTI_INST_ENTRY_ID_HVSP2,privatedata ,enType,stCfg);

    if(enMultiInstRet == E_MDRV_MULTI_INST_STATUS_LOCKED)
    {
        ret = -EINVAL;
    }
    else if (enMultiInstRet == E_MDRV_MULTI_INST_STATUS_FAIL)
    {
        ret = -EFAULT;
    }
    else
    {
        ret = 0;
    }
    return ret;
}

int _mdrv_ms_hvsp2_io_set_input_config(struct file *filp, unsigned long arg)
{
    ST_MDRV_HVSP_INPUT_CONFIG stInCfg;
    ST_IOCTL_HVSP_INPUT_CONFIG stIOInCfg;
    int ret = 0;
    ST_MDRV_HVSP_VERSIONCHK_CONFIG stVersion;

     _mdrv_ms_hvsp2_io_fill_versionchkstruct(sizeof(ST_IOCTL_HVSP_INPUT_CONFIG),
        (((ST_IOCTL_HVSP_INPUT_CONFIG __user *)arg)->VerChk_Size),
        &(((ST_IOCTL_HVSP_INPUT_CONFIG __user *)arg)->VerChk_Version),&stVersion);
    if(_mdrv_ms_hvsp2_io_version_check(&stVersion))
    {
        SCL_ERR( "[HVSP2]   %s  \n", __FUNCTION__);
        return -EINVAL;
    }
    else
    {
        if(copy_from_user(&stIOInCfg, (__user ST_IOCTL_HVSP_INPUT_CONFIG  *)arg, sizeof(ST_IOCTL_HVSP_INPUT_CONFIG)))
        {
            return -EFAULT;
        }
        else
        {
            stInCfg.enColor       = (EN_MDRV_HVSP_COLOR_TYPE)stIOInCfg.enColor;
            stInCfg.enSrcType     = (EN_MDRV_HVSP_SRC_TYPE)stIOInCfg.enSrcType;
            memcpy(&stInCfg.stCaptureWin , &stIOInCfg.stCaptureWin,sizeof(ST_MDRV_HVSP_WINDOW_CONFIG));
            memcpy(&stInCfg.stTimingCfg , &stIOInCfg.stTimingCfg,sizeof(ST_MDRV_HVSPTIMING_CONFIG));
        }

    }

    if(_ms_hvsp2_multiinstSet(E_MDRV_MULTI_INST_CMD_HVSP_IN_CONFIG, (void *)&stInCfg, filp->private_data))
    {
        ret = -EFAULT;
    }
    else
    {
        if(!MDrv_HVSP_SetInputConfig(E_MDRV_HVSP_ID_2,  &stInCfg))
        {
            ret = -EFAULT;
        }
        else
        {
            ret = 0;
        }
    }

    return ret;

}

int _mdrv_ms_hvsp2_io_set_output_config(struct file *filp, unsigned long arg)
{
    ST_IOCTL_HVSP_OUTPUT_CONFIG stOutCfg;
    int ret = 0;

    ST_MDRV_HVSP_VERSIONCHK_CONFIG stVersion;

     _mdrv_ms_hvsp2_io_fill_versionchkstruct(sizeof(ST_IOCTL_HVSP_OUTPUT_CONFIG),
        (((ST_IOCTL_HVSP_OUTPUT_CONFIG __user *)arg)->VerChk_Size),
        &(((ST_IOCTL_HVSP_OUTPUT_CONFIG __user *)arg)->VerChk_Version),&stVersion);
    if(_mdrv_ms_hvsp2_io_version_check(&stVersion))
    {
        SCL_ERR( "[HVSP2]   %s  \n", __FUNCTION__);
        return -EINVAL;
    }
    else
    {
        if(copy_from_user(&stOutCfg, (__user ST_IOCTL_HVSP_OUTPUT_CONFIG  *)arg, sizeof(ST_IOCTL_HVSP_OUTPUT_CONFIG)))
        {
            return -EFAULT;
        }
    }

    return ret;
}


int _mdrv_ms_hvsp2_io_set_scaling_config(struct file *filp, unsigned long arg)
{
    ST_IOCTL_HVSP_SCALING_CONFIG stIOSclCfg;
    ST_MDRV_HVSP_SCALING_CONFIG stSclCfg;
    int ret = 0;

    ST_MDRV_HVSP_VERSIONCHK_CONFIG stVersion;

     _mdrv_ms_hvsp2_io_fill_versionchkstruct(sizeof(ST_IOCTL_HVSP_SCALING_CONFIG),
        (((ST_IOCTL_HVSP_SCALING_CONFIG __user *)arg)->VerChk_Size),
        &(((ST_IOCTL_HVSP_SCALING_CONFIG __user *)arg)->VerChk_Version),&stVersion);
    if(_mdrv_ms_hvsp2_io_version_check(&stVersion))
    {
        SCL_ERR( "[HVSP2]   %s  \n", __FUNCTION__);
        return -EINVAL;
    }
    else
    {
        if(copy_from_user(&stIOSclCfg, (__user ST_IOCTL_HVSP_SCALING_CONFIG  *)arg, sizeof(ST_IOCTL_HVSP_SCALING_CONFIG)))
        {
            return -EFAULT;
        }
        else
        {
            stSclCfg.stclk     = (ST_MDRV_HVSP_CLK_CONFIG *)&(_dev_ms_hvsp2.stclk);
            stSclCfg.stCropWin.bEn = stIOSclCfg.bCropEn;
            stSclCfg.stCropWin.u16Height = stIOSclCfg.stCropWin.u16Height;
            stSclCfg.stCropWin.u16Width = stIOSclCfg.stCropWin.u16Width;
            stSclCfg.stCropWin.u16X = stIOSclCfg.stCropWin.u16X;
            stSclCfg.stCropWin.u16Y = stIOSclCfg.stCropWin.u16Y;
            stSclCfg.u16Dsp_Height = stIOSclCfg.u16Dsp_Height;
            stSclCfg.u16Dsp_Width = stIOSclCfg.u16Dsp_Width;
            stSclCfg.u16Src_Height = stIOSclCfg.u16Src_Height;
            stSclCfg.u16Src_Width = stIOSclCfg.u16Src_Width;
        }
    }
    if(_ms_hvsp2_multiinstSet(E_MDRV_MULTI_INST_CMD_HVSP_SCALING_CONFIG, (void *)&stSclCfg, filp->private_data))
    {
        ret = -EFAULT;
    }
    else
    {
        if(!MDrv_HVSP_SetScalingConfig(E_MDRV_HVSP_ID_2,  &stSclCfg))
        {
            ret = -EFAULT;
        }
        else
        {
            ret = 0;
        }
    }

    return ret;
}

int _mdrv_ms_hvsp2_io_get_private_id_config(struct file *filp, unsigned long arg)
{
    ST_IOCTL_HVSP_PRIVATE_ID_CONFIG stCfg;

    if(!MDrv_MultiInst_Entry_GetPirvateId(E_MDRV_MULTI_INST_ENTRY_ID_HVSP2, filp->private_data, &stCfg.s32Id))
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_IOCTL_HVSP_PRIVATE_ID_CONFIG __user *)arg, &stCfg, sizeof(ST_IOCTL_HVSP_PRIVATE_ID_CONFIG)))
    {
       return -EFAULT;
    }

    return 0;
}

int _mdrv_ms_hvsp2_io_get_inform_config(struct file *filp, unsigned long arg)
{
    ST_IOCTL_HVSP_SCINFORM_CONFIG stIOInfoCfg;
    ST_MDRV_HVSP_SCINFORM_CONFIG stInfoCfg;
    int ret = 0;
    if(!MDrv_HVSP_GetSCLInform(E_MDRV_HVSP_ID_2,  &stInfoCfg))
    {
        ret = -EFAULT;
    }
    else
    {
        memcpy(&stIOInfoCfg, &stInfoCfg, sizeof(ST_IOCTL_HVSP_SCINFORM_CONFIG));
        if(copy_to_user((ST_IOCTL_HVSP_SCINFORM_CONFIG __user *)arg, &stIOInfoCfg, sizeof(ST_IOCTL_HVSP_SCINFORM_CONFIG)))
        {
            ret = -EFAULT;
        }
        else
        {
            ret = 0;
        }
    }

    return ret;
}

int _mdrv_ms_hvsp2_io_get_version(struct file *filp, unsigned long arg)
{
    int ret = 0;

    if (CHK_VERCHK_HEADER( &(((ST_IOCTL_HVSP_VERSION_CONFIG __user *)arg)->VerChk_Version)) )
    {
        if( CHK_VERCHK_VERSION_LESS( &(((ST_IOCTL_HVSP_VERSION_CONFIG __user *)arg)->VerChk_Version), IOCTL_HVSP_VERSION) )
        {

            VERCHK_ERR("[HVSP] Version(%04x) < %04x!!! \n",
                ((ST_IOCTL_HVSP_VERSION_CONFIG __user *)arg)->VerChk_Version & VERCHK_VERSION_MASK,
                IOCTL_HVSP_VERSION);

            ret = -EINVAL;
        }
        else
        {
            if( CHK_VERCHK_SIZE( &(((ST_IOCTL_HVSP_VERSION_CONFIG __user *)arg)->VerChk_Size), sizeof(ST_IOCTL_HVSP_VERSION_CONFIG)) == 0 )
            {
                VERCHK_ERR("[HVSP] Size(%04x) != %04x!!! \n",
                    sizeof(ST_IOCTL_HVSP_VERSION_CONFIG),
                    (((ST_IOCTL_HVSP_VERSION_CONFIG __user *)arg)->VerChk_Size));

                ret = -EINVAL;
            }
            else
            {
                ST_IOCTL_HVSP_VERSION_CONFIG stCfg;

                stCfg = FILL_VERCHK_TYPE(stCfg, stCfg.VerChk_Version, stCfg.VerChk_Size, IOCTL_HVSP_VERSION);
                stCfg.u32Version = IOCTL_HVSP_VERSION;

                if(copy_to_user((ST_IOCTL_HVSP_VERSION_CONFIG __user *)arg, &stCfg, sizeof(ST_IOCTL_HVSP_VERSION_CONFIG)))
                {
                    ret = -EFAULT;
                }
                else
                {
                    ret = 0;
                }
            }
        }
    }
    else
    {
        VERCHK_ERR("[HVSP] No Header !!! \n");
        SCL_ERR( "[HVSP]%s  \n", __FUNCTION__);
        ret = -EINVAL;
    }

    return ret;
}
int _mdrv_ms_hvsp2_io_set_osd_config(struct file *filp, unsigned long arg)
{
    ST_MDRV_HVSP_OSD_CONFIG stOSDCfg;
    ST_IOCTL_HVSP_OSD_CONFIG stIOOSDCfg;
    int ret = 0;
    ST_MDRV_HVSP_VERSIONCHK_CONFIG stVersion;

    _mdrv_ms_hvsp2_io_fill_versionchkstruct(sizeof(ST_IOCTL_HVSP_OSD_CONFIG),
        (((ST_IOCTL_HVSP_OSD_CONFIG __user *)arg)->VerChk_Size),
        &(((ST_IOCTL_HVSP_OSD_CONFIG __user *)arg)->VerChk_Version),&stVersion);
    if(_mdrv_ms_hvsp2_io_version_check(&stVersion))
    {
        SCL_ERR( "[HVSP2]   %s  \n", __FUNCTION__);
        return -EINVAL;
    }
    else
    {
        if(copy_from_user(&stIOOSDCfg, (__user ST_IOCTL_HVSP_OSD_CONFIG  *)arg, sizeof(ST_IOCTL_HVSP_OSD_CONFIG)))
        {
            return -EFAULT;
        }
        else
        {
            stOSDCfg.enOSD_loc = (EN_MDRV_HVSP_OSD_LOC_TYPE)stIOOSDCfg.enOSD_loc;
            stOSDCfg.stOsdOnOff.bOSDEn = stIOOSDCfg.bEn;
        }
    }
    if(_ms_hvsp2_multiinstSet(E_MDRV_MULTI_INST_CMD_HVSP_SET_OSD_CONFIG, (void *)&stOSDCfg, filp->private_data))
    {
        ret = -EFAULT;
    }
    else
    {
        if(!MDrv_HVSP_SetOSDConfig(E_MDRV_HVSP_ID_3,  &stOSDCfg))
        {
            ret = -EFAULT;
        }
        else
        {
            ret = 0;
        }
    }
    return ret;
}
//----------------------------------------------------------------------------------------------


//==============================================================================
long mdrv_ms_hvsp2_ioctl(struct file *filp, unsigned int u32Cmd, unsigned long u32Arg)
{
    int err = 0;
    int retval = 0;

    if(_dev_ms_hvsp2.refCnt <= 0)
    {
        SCL_ERR( "[HVSP2] HVSP2IO_IOCTL refCnt =%d!!! \n", _dev_ms_hvsp2.refCnt);
        return -EFAULT;
    }
    /* check u32Cmd valid */
    if(IOCTL_HVSP_MAGIC == _IOC_TYPE(u32Cmd))
    {
        if(_IOC_NR(u32Cmd) >= IOCTL_HVSP_MAX_NR)
        {
            SCL_ERR( "[HVSP2] IOCtl NR Error!!! (Cmd=%x)\n",u32Cmd);
            return -ENOTTY;
        }
    }
    else
    {
        SCL_ERR( "[HVSP2] IOCtl MAGIC Error!!! (Cmd=%x)\n",u32Cmd);
        return -ENOTTY;
    }

    /* verify Access */
    if (_IOC_DIR(u32Cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)u32Arg, _IOC_SIZE(u32Cmd));
    }
    else if (_IOC_DIR(u32Cmd) & _IOC_WRITE)
    {
        err =  !access_ok(VERIFY_READ, (void __user *)u32Arg, _IOC_SIZE(u32Cmd));
    }
    if (err)
    {
        return -EFAULT;
    }
	/* not allow query or command once driver suspend */

    SCL_DBG(SCL_DBG_LV_IOCTL()&EN_DBGMG_IOCTLEVEL_SC2, "[HVSP2] IOCTL_NUM:: == %s ==  \n", (CMD_PARSING(u32Cmd)));

    switch(u32Cmd)
    {
    case IOCTL_HVSP_SET_IN_CONFIG:
        retval = _mdrv_ms_hvsp2_io_set_input_config(filp, u32Arg);
        break;

    case IOCTL_HVSP_SET_OUT_CONFIG:
        retval = _mdrv_ms_hvsp2_io_set_output_config(filp, u32Arg);
        break;

    case IOCTL_HVSP_SET_SCALING_CONFIG:
        retval = _mdrv_ms_hvsp2_io_set_scaling_config(filp, u32Arg);
        break;

    case IOCTL_HVSP_GET_PRIVATE_ID_CONFIG:
        retval = _mdrv_ms_hvsp2_io_get_private_id_config(filp, u32Arg);
        break;

    case IOCTL_HVSP_SET_OSD_CONFIG:
        retval = _mdrv_ms_hvsp2_io_set_osd_config(filp, u32Arg);
        break;
    case IOCTL_HVSP_SET_FB_MANAGE_CONFIG:
    case IOCTL_HVSP_REQ_MEM_CONFIG:
    case IOCTL_HVSP_SET_MISC_CONFIG:
    case IOCTL_HVSP_SET_POST_CROP_CONFIG:
        SCL_ERR( "[HVSP2] Not Support IOCTL %x\n ",u32Cmd);
        retval = -EINVAL;
        break;
    case IOCTL_HVSP_GET_INFORM_CONFIG:
        retval = _mdrv_ms_hvsp2_io_get_inform_config(filp, u32Arg);
        break;

    case IOCTL_HVSP_GET_VERSION_CONFIG:
        retval = _mdrv_ms_hvsp2_io_get_version(filp, u32Arg);
        break;

    default:  /* redundant, as cmd was checked against MAXNR */
        SCL_ERR( "[HVSP2] ERROR IOCtl number %x\n ",u32Cmd);
        retval = -ENOTTY;
        break;
    }

    return retval;
}


static unsigned int mdrv_ms_hvsp2_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int ret = 0;
    EN_MDRV_MULTI_INST_STATUS_TYPE enMultiInstRet;
    wait_queue_head_t *pWaitQueueHead = NULL;
    enMultiInstRet = MDrv_MultiInst_Etnry_IsFree(E_MDRV_MULTI_INST_ENTRY_ID_HVSP2, filp->private_data);
    SCL_DBG(SCL_DBG_LV_IOCTL()&EN_DBGMG_IOCTLEVEL_SC2, "[SCLDMA1]start %s ret=%x\n",__FUNCTION__,ret);
    if(enMultiInstRet == E_MDRV_MULTI_INST_STATUS_SUCCESS)
    {
        pWaitQueueHead = (wait_queue_head_t *)MDrv_HVSP_GetWaitQueueHead();
        MDrv_HVSP_SetPollWait(filp, pWaitQueueHead, wait);
        if(gu8sc2first)
        {
            ret = POLLIN;
            gu8sc2first = 0;
        }
        else if(MDrv_HVSP_GetCMDQDoneStatus(E_MDRV_HVSP_ID_2))
        {
            ret = POLLIN;
        }
        else
        {
            ret = 0;
        }
    }
    else
    {
        ret = 0;
    }
    return ret;
}
#if CONFIG_OF
static int mdrv_ms_hvsp2_probe(struct platform_device *pdev)
{
    ST_MDRV_HVSP_INIT_CONFIG stHVSPInitCfg;
    int s32Ret;
    dev_t  dev;
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s:%d\n",__FUNCTION__,__LINE__);
//mod init
    if(_dev_ms_hvsp2.s32Major)
    {
        dev     = MKDEV(_dev_ms_hvsp2.s32Major, _dev_ms_hvsp2.s32Minor);
        s32Ret  = register_chrdev_region(dev, MDRV_MS_HVSP_DEVICE_COUNT, MDRV_MS_HVSP_NAME);
    }
    else
    {
        s32Ret                  = alloc_chrdev_region(&dev, _dev_ms_hvsp2.s32Minor, MDRV_MS_HVSP_DEVICE_COUNT, MDRV_MS_HVSP_NAME);
        _dev_ms_hvsp2.s32Major  = MAJOR(dev);
    }

    if (0 > s32Ret)
    {
        SCL_ERR( "[HVSP2] Unable to get major %d\n", _dev_ms_hvsp2.s32Major);
        return s32Ret;
    }

    cdev_init(&_dev_ms_hvsp2.cdev, &_dev_ms_hvsp2.fops);
    if (0 != (s32Ret= cdev_add(&_dev_ms_hvsp2.cdev, dev, MDRV_MS_HVSP_DEVICE_COUNT)))
    {
        SCL_ERR( "[HVSP2] Unable add a character device\n");
        unregister_chrdev_region(dev, MDRV_MS_HVSP_DEVICE_COUNT);
        return s32Ret;
    }

    m_hvsp2_class = msys_get_sysfs_class();
    if(!m_hvsp2_class)
    {
        m_hvsp2_class = class_create(THIS_MODULE, hvsp2_classname);
    }
    if(IS_ERR(m_hvsp2_class))
    {
        printk(KERN_WARNING"Failed at class_create().Please exec [mknod] before operate the device/n");
    }
    else
    {
        _dev_ms_hvsp2.devicenode = device_create(m_hvsp2_class, NULL, dev,NULL, "mhvsp2");
        _dev_ms_hvsp2.devicenode->dma_mask=&ms_hvsp2_dma_mask;
        _dev_ms_hvsp2.devicenode->coherent_dma_mask=ms_hvsp2_dma_mask;
    }

//probe
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);
    MsOS_SetSclIrqIDFormSys(pdev,0,E_SCLIRQ_SC0);
    MsOS_SetCmdqIrqIDFormSys(pdev,1,E_CMDQIRQ_CMDQ0);
    stHVSPInitCfg.u32Riubase    = 0x1F000000; //ToDo
    stHVSPInitCfg.u32IRQNUM     = MsOS_GetIrqIDSCL(E_SCLIRQ_SC0);
    stHVSPInitCfg.u32CMDQIRQNUM = MsOS_GetIrqIDCMDQ(E_CMDQIRQ_CMDQ0);
    if( MDrv_HVSP_Init(E_MDRV_HVSP_ID_2, &stHVSPInitCfg) == 0)
    {
        return -EFAULT;
    }
    //clk enable
    st_ms_hvsp2_device.dev.of_node = pdev->dev.of_node;
    _dev_ms_hvsp2.stclk.idclk = of_clk_get(st_ms_hvsp2_device.dev.of_node,0);
    _dev_ms_hvsp2.stclk.fclk1 = of_clk_get(st_ms_hvsp2_device.dev.of_node,1);
    _dev_ms_hvsp2.stclk.fclk2 = of_clk_get(st_ms_hvsp2_device.dev.of_node,2);
    _dev_ms_hvsp2.stclk.odclk = of_clk_get(st_ms_hvsp2_device.dev.of_node,3);
    if (IS_ERR(_dev_ms_hvsp2.stclk.idclk) || IS_ERR(_dev_ms_hvsp2.stclk.fclk1)
        || IS_ERR(_dev_ms_hvsp2.stclk.fclk2)|| IS_ERR(_dev_ms_hvsp2.stclk.odclk))
    {
        SCL_ERR( "[HVSP2] Can't Get CLK\n");
        return 0 ;
    }
    device_create_file(_dev_ms_hvsp2.devicenode, &dev_attr_osd);
    device_create_file(_dev_ms_hvsp2.devicenode, &dev_attr_SCIQ);
    //sysfs_create_link(&pdev->dev.parent->kobj, &pdev->dev.kobj, "mhvsp2");
    MDrv_MultiInst_Entry_Init_Variable(E_MDRV_MULTI_INST_ENTRY_ID_HVSP2);
    gbProbeAlready |= EN_DBG_HVSP2_CONFIG;
    gu8sc2first = 1;
    return 0;
}
static int mdrv_ms_hvsp2_remove(struct platform_device *pdev)
{
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);
    gbProbeAlready = (gbProbeAlready&(~EN_DBG_HVSP2_CONFIG));
    if(gbProbeAlready == 0)
    {
        MDrv_HVSP_Exit(1);
    }
    else
    {
        MDrv_HVSP_Exit(0);
    }
    cdev_del(&_dev_ms_hvsp2.cdev);
    device_destroy(m_hvsp2_class, MKDEV(_dev_ms_hvsp2.s32Major, _dev_ms_hvsp2.s32Minor));
    class_destroy(m_hvsp2_class);
    unregister_chrdev_region(MKDEV(_dev_ms_hvsp2.s32Major, _dev_ms_hvsp2.s32Minor), MDRV_MS_HVSP_DEVICE_COUNT);
    return 0;
}

#else
static int mdrv_ms_hvsp2_probe(struct platform_device *pdev)
{
    ST_MDRV_HVSP_INIT_CONFIG stHVSPInitCfg;

    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);

    stHVSPInitCfg.u32Riubase = 0x1F000000; //ToDo

    if( MDrv_HVSP_Init(E_MDRV_HVSP_ID_2, &stHVSPInitCfg) == 0)
    {
        return -EFAULT;
    }
    MDrv_MultiInst_Entry_Init_Variable(E_MDRV_MULTI_INST_ENTRY_ID_HVSP2);

    return 0;
}

static int mdrv_ms_hvsp2_remove(struct platform_device *pdev)
{
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);

    return 0;
}
#endif
static int mdrv_ms_hvsp2_suspend(struct platform_device *dev, pm_message_t state)
{
    ST_MDRV_HVSP_SUSPEND_RESUME_CONFIG stHvspSuspendResumeCfg;
    int ret = 0;
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);

    stHvspSuspendResumeCfg.u32IRQNum = 0xFFFFFFFF;
    stHvspSuspendResumeCfg.u32CMDQIRQNum = 0xFFFFFFFF;
    if(MDrv_HVSP_Suspend(E_MDRV_HVSP_ID_2, &stHvspSuspendResumeCfg))
    {
        ret = 0;
    }
    else
    {
        ret = -EFAULT;
    }

    return ret;
}

static int mdrv_ms_hvsp2_resume(struct platform_device *dev)
{
    EN_MDRV_MULTI_INST_STATUS_TYPE enMultiInstRet ;
    ST_MDRV_HVSP_SUSPEND_RESUME_CONFIG stHvspSuspendResumeCfg;
    int ret = 0;

    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);

    stHvspSuspendResumeCfg.u32IRQNum = 0xFFFFFFFF;
    stHvspSuspendResumeCfg.u32CMDQIRQNum = 0xFFFFFFFF;
    if(MDrv_HVSP_Resume(E_MDRV_HVSP_ID_2, &stHvspSuspendResumeCfg))
    {
        enMultiInstRet = MDrv_MultiInst_Entry_FlashData(
                            E_MDRV_MULTI_INST_ENTRY_ID_HVSP2,
                            NULL,
                            E_MDRV_MULTI_INST_CMD_FORCE_RELOAD_CONFIG,
                            NULL);

        if(enMultiInstRet != E_MDRV_MULTI_INST_STATUS_SUCCESS)
        {
            SCL_ERR( "[HVSP1] %s: Fail\n",__FUNCTION__);
            ret = -EINVAL;
        }
        else
        {
            ret = 0;
        }
    }
    else
    {
        SCL_ERR( "[HVSP1] %s: Fail\n",__FUNCTION__);
        ret = -EFAULT;
    }

    return ret;
}


int mdrv_ms_hvsp2_open(struct inode *inode, struct file *filp)
{
    int ret = 0;
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);

    SCL_ASSERT(_dev_ms_hvsp2.refCnt>=0);

    if(filp->private_data == NULL)
    {
        if(MDrv_MultiInst_Entry_Alloc(E_MDRV_MULTI_INST_ENTRY_ID_HVSP2, &filp->private_data) == 0)
        {
            ret =  -EFAULT;
        }
    }
    if(!ret)
    {
        _dev_ms_hvsp2.refCnt++;
    }

    return ret;
}


int mdrv_ms_hvsp2_release(struct inode *inode, struct file *filp)
{

    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);

    MDrv_MultiInst_Entry_Free(E_MDRV_MULTI_INST_ENTRY_ID_HVSP2, filp->private_data);

    _dev_ms_hvsp2.refCnt--;
    SCL_ASSERT(_dev_ms_hvsp2.refCnt >= 0);
    if(_dev_ms_hvsp2.refCnt == 0)
    {
        MDrv_HVSP_Release(E_MDRV_HVSP_ID_2);
        gu8sc2first = 1;
    }
    //free_irq(INT_IRQ_HVSP2W, MDrv_HVSP2W_isr);
    return 0;
}


//-------------------------------------------------------------------------------------------------
// Module functions
//-------------------------------------------------------------------------------------------------
#if CONFIG_OF
int _mdrv_ms_hvsp2_init(void)
{
    int ret = 0;
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s:%d\n",__FUNCTION__,__LINE__);
    ret = platform_driver_register(&st_ms_hvsp2_driver);
    if (!ret)
    {
        SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] platform_driver_register success\n");
        if(gbProbeAlready&EN_DBG_HVSP2_CONFIG)
        {
            SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] Probe success\n");
        }
        else
        {
            SCL_ERR( "[HVSP2] Probe Fail\n");
        }
        if(gbProbeAlready==EN_DBG_SCL_CONFIG)
        {
            SCL_ERR( "[SCL] SCL init success\n");
        }
    }
    else
    {
        SCL_ERR( "[HVSP2] platform_driver_register failed\n");
        platform_driver_unregister(&st_ms_hvsp2_driver);
    }


    return ret;
}
void _mdrv_ms_hvsp2_exit(void)
{
    /*de-initial the who GFLIPDriver */
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);
    platform_driver_unregister(&st_ms_hvsp2_driver);
}
#else
int _mdrv_ms_hvsp2_init(void)
{
    int ret = 0;
    int s32Ret;
    dev_t  dev;
    struct device_node *np = NULL;
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);

    if(_dev_ms_hvsp2.s32Major)
    {
        dev     = MKDEV(_dev_ms_hvsp2.s32Major, _dev_ms_hvsp2.s32Minor);
        s32Ret  = register_chrdev_region(dev, MDRV_MS_HVSP_DEVICE_COUNT, MDRV_MS_HVSP_NAME);
    }
    else
    {
        s32Ret                  = alloc_chrdev_region(&dev, _dev_ms_hvsp2.s32Minor, MDRV_MS_HVSP_DEVICE_COUNT, MDRV_MS_HVSP_NAME);
        _dev_ms_hvsp2.s32Major  = MAJOR(dev);
    }

    if (0 > s32Ret)
    {
        SCL_ERR( "[HVSP2] Unable to get major %d\n", _dev_ms_hvsp2.s32Major);
        return s32Ret;
    }

    cdev_init(&_dev_ms_hvsp2.cdev, &_dev_ms_hvsp2.fops);
    if (0 != (s32Ret= cdev_add(&_dev_ms_hvsp2.cdev, dev, MDRV_MS_HVSP_DEVICE_COUNT)))
    {
        SCL_ERR( "[HVSP2] Unable add a character device\n");
        unregister_chrdev_region(dev, MDRV_MS_HVSP_DEVICE_COUNT);
        return s32Ret;
    }

    m_hvsp2_class = class_create(THIS_MODULE, hvsp2_classname);
    if(IS_ERR(m_hvsp2_class))
    {
        printk(KERN_WARNING"Failed at class_create().Please exec [mknod] before operate the device/n");
    }
    else
    {
        device_create(m_hvsp2_class, NULL, dev,NULL, "mhvsp2");
    }

    /* initial the whole HVSP2 Driver */
    ret = platform_driver_register(&st_ms_hvsp2_driver);

    if (!ret)
    {
        ret = platform_device_register(&st_ms_hvsp2_device);
        if (ret)    /* if register device fail, then unregister the driver.*/
        {
            platform_driver_unregister(&st_ms_hvsp2_driver);
            SCL_ERR( "[HVSP2] platform_driver_register failed\n");

        }
        else
        {
            SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] platform_driver_register success\n");
        }
    }


    return ret;
}
void _mdrv_ms_hvsp2_exit(void)
{
    /*de-initial the who GFLIPDriver */
    SCL_DBG(SCL_DBG_LV_MDRV_IO(), "[HVSP2] %s\n",__FUNCTION__);

    cdev_del(&_dev_ms_hvsp2.cdev);
    device_destroy(m_hvsp2_class, MKDEV(_dev_ms_hvsp2.s32Major, _dev_ms_hvsp2.s32Minor));
    class_destroy(m_hvsp2_class);
    unregister_chrdev_region(MKDEV(_dev_ms_hvsp2.s32Major, _dev_ms_hvsp2.s32Minor), MDRV_MS_HVSP_DEVICE_COUNT);
    platform_driver_unregister(&st_ms_hvsp2_driver);
}
#endif

module_init(_mdrv_ms_hvsp2_init);
module_exit(_mdrv_ms_hvsp2_exit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("ms hvsp2 ioctrl driver");
MODULE_LICENSE("GPL");
