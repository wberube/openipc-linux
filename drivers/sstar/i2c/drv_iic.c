/*
 * drv_iic.c- Sigmastar
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

#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <ms_platform.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
#include "../include/ms_msys.h"

#include "hal_iic.h"
#include "hal_iic_reg.h"
#include "cam_os_wrapper.h"
#include "cam_sysfs.h"

/*********debug mesg*********/

//#define DMSG_I2C_DRIVER_DEBUG
#define dmsg_i2c_drverr(fmt, ...)                                        \
    do                                                                   \
    {                                                                    \
        CamOsPrintf("[drv_i2c_err] <%s> " fmt, __func__, ##__VA_ARGS__); \
    } while (0)
#ifdef DMSG_I2C_DRIVER_DEBUG
#define dmsg_i2c_drvwarn(fmt, ...)                                        \
    do                                                                    \
    {                                                                     \
        CamOsPrintf("[drv_i2c_warn] <%s> " fmt, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define dmsg_i2c_drvwarn(fmt, ...)
#endif
/*******************************************************************/
/*                    value type                                   */
/*******************************************************************/
typedef struct _sstar_iic
{
    u32                u32IrqNum;
    u8                 u8IrqName[20];
    void *             pvI2cClk;
    ST_HAL_I2C_BASE    stHalCtrl;
    struct device *    pDevice;
    struct i2c_adapter stAdapter;
} st_i2c_ctrl;

/*******************************************************************/
/*                    global value definition                      */
/*******************************************************************/

/*******************************************************************/
/*                    function definition                          */
/*******************************************************************/

/****************************************************/
//*func:
//*description:
//*parameter:
//*return:
/****************************************************/
static s32 sstar_i2c_set_srclk(void *para_i2c_base, u32 para_src_clk)
{
    s32         s32Ret = 0;
    struct clk *pstI2cClk;

    st_i2c_ctrl *           pstI2cCtrl;
    ST_HAL_I2C_BASE *       pstI2cBase;
    struct platform_device *pDevice;

#ifdef CONFIG_CAM_CLK
    u32                  u32camclk;
    CAMCLK_Set_Attribute stSetCfg;
    CAMCLK_Get_Attribute stGetCfg;
#else
#endif

    /****common code start*****/
    pstI2cBase = para_i2c_base;

    pstI2cCtrl = container_of(pstI2cBase, struct _sstar_iic, stHalCtrl);
    if (!pstI2cCtrl)
    {
        dmsg_i2c_drverr("get i2c ctrl pointer err\n");
        return -EFAULT;
    }

    pDevice = container_of(pstI2cCtrl->pDevice, struct platform_device, dev);
    if (!pDevice)
    {
        dmsg_i2c_drverr("get i2c-%d platform device pointer err\n", pstI2cCtrl->stAdapter.nr);
        return -EFAULT;
    }
    /*****common code end******/

#ifdef CONFIG_CAM_CLK
    of_property_read_u32_index(pDevice->dev.of_node, "camclk", &u32camclk) if (!u32camclk)
    {
        dmsg_i2c_drverr("find i2c-%d camclk failure\n", pstI2cCtrl->stAdapter.nr);
        s32Ret = -ENOENT;
        goto out;
    }
    if (CamClkRegister("iic", u32camclk, &(pstI2cCtrl->pvI2cClk)) == CAMCLK_RET_OK)
    {
        CamClkAttrGet(pstI2cCtrl->pvI2cClk, &stGetCfg);
        CAMCLK_SETPARENT(stSetCfg, stGetCfg.u32Parent[0]);
        CamClkAttrSet(pstI2cCtrl->pvI2cClk, &stSetCfg);
        CamClkSetOnOff(pstI2cCtrl->pvI2cClk, 1);
    }
#else // else CONFIG_CAM_CLK(no define)

    pstI2cClk = of_clk_get(pDevice->dev.of_node, 0);
    if (IS_ERR(pstI2cClk))
    {
        dmsg_i2c_drverr("i2c-%d of_clk_get err!\n", pstI2cCtrl->stAdapter.nr);
        s32Ret = -ENOENT;
        goto out;
    }
    clk_set_rate(pstI2cClk, para_src_clk);

#endif // end of CONFIG_CAM_CLK

out:

#ifndef CONFIG_CAM_CLK
    clk_put(pstI2cClk);
#endif
    return s32Ret;
}

/****************************************************/
//*func:
//*description:
//*parameter:
//*return:
/****************************************************/
static s32 sstar_i2c_init(st_i2c_ctrl *para_i2c_ctrl, struct platform_device *para_pdev)
{
    s32              s32Ret;
    ST_HAL_DMA_ADDR *pstDmaAddr;
    MSYS_DMEM_INFO   iic_mem_info;

    pstDmaAddr = &para_i2c_ctrl->stHalCtrl.stI2cDmaCtrl.stDmaMiuAdr;
    if (para_i2c_ctrl->stHalCtrl.u32EnDma)
    {
        iic_mem_info.length = 4096;
        scnprintf(iic_mem_info.name, sizeof(iic_mem_info.name), "Sstar IIC %d DMA", para_i2c_ctrl->stAdapter.nr);
        s32Ret = msys_request_dmem(&iic_mem_info);
        if (s32Ret)
        {
            dmsg_i2c_drverr("i2c-%d alloc mem for dma failed\n", para_i2c_ctrl->stAdapter.nr);
            goto out;
        }
        pstDmaAddr->pu8DmaAdrVirtu = iic_mem_info.kvirt;
        pstDmaAddr->u64DmaAdrPhy   = iic_mem_info.phys;
        pstDmaAddr->u64DmaAdrMiu   = (u64)Chip_Phys_to_MIU((ss_phys_addr_t)pstDmaAddr->u64DmaAdrPhy);
        dmsg_i2c_drvwarn("<<<<<<<<<<<<<<<IIC DMA PHYS ADDR IS %#llx, MIU ADDR IS: %#llx\n", pstDmaAddr->u64DmaAdrPhy,
                         pstDmaAddr->u64DmaAdrMiu);
    }
    s32Ret = HAL_I2C_Init(&para_i2c_ctrl->stHalCtrl);
out:
    return s32Ret;
}

/****************************************************/
//*func:
//*description:
//*parameter:
//*return:
/****************************************************/
static s32 sstar_i2c_deinit(st_i2c_ctrl *para_i2c_ctrl)
{
    ST_HAL_DMA_ADDR *pstDmaAddr;
    MSYS_DMEM_INFO   iic_mem_info;

    pstDmaAddr = &para_i2c_ctrl->stHalCtrl.stI2cDmaCtrl.stDmaMiuAdr;
    if (para_i2c_ctrl->stHalCtrl.u32EnDma)
    {
        iic_mem_info.length = 4096;
        scnprintf(iic_mem_info.name, sizeof(iic_mem_info.name), "Sstar IIC %d DMA", para_i2c_ctrl->stAdapter.nr);
        iic_mem_info.kvirt = pstDmaAddr->pu8DmaAdrVirtu;
        iic_mem_info.phys  = pstDmaAddr->u64DmaAdrPhy;
        msys_release_dmem(&iic_mem_info);
        pstDmaAddr->u64DmaAdrMiu = 0x0;
    }
    CamOsTsemDeinit(&para_i2c_ctrl->stHalCtrl.stTsemID);
    i2c_del_adapter(&para_i2c_ctrl->stAdapter);

    return 0;
}
static u32 sstar_i2c_func(struct i2c_adapter *padapter)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/****************************************************/
//*func:
//*description:
//*parameter:
//*return:
/****************************************************/
s32 sstar_i2c_master_xfer(struct i2c_adapter *para_adapter, struct i2c_msg *para_msg, s32 para_num)
{
    s32             s32Ret;
    s32             l_num;
    st_i2c_ctrl *   pstI2cCtrl = para_adapter->dev.driver_data;
    struct i2c_msg *pstI2cMsg  = para_msg;
#ifndef CONFIG_CAM_CLK
    struct clk *pstI2cClk = NULL;
#endif

    CamOsTsemDown(&pstI2cCtrl->stHalCtrl.stTsemID);
#ifdef CONFIG_CAM_CLK

#else
    pstI2cClk = of_clk_get(para_adapter->dev.of_node, 0);
    clk_prepare_enable(pstI2cClk);
#endif

    for (l_num = 0; l_num < para_num; l_num++, pstI2cMsg++)
    {
        // dma need set stop format before transfer
        if (pstI2cCtrl->stHalCtrl.u32EnDma)
        {
            if (!(pstI2cMsg->flags & 0x02) && (para_num > 1))
            {
                HAL_I2C_DmaStopFmt(&pstI2cCtrl->stHalCtrl, ((pstI2cMsg->flags) & I2C_M_RD));
            }
            else
            {
                HAL_I2C_DmaStopFmt(&pstI2cCtrl->stHalCtrl, 1);
            }
        }

        dmsg_i2c_drvwarn("para number from user is : %d,flags is : 0x%x\n", para_num, pstI2cMsg->flags);
        if ((pstI2cMsg->flags) & I2C_M_RD)
        {
            dmsg_i2c_drvwarn("IN READ \n");
            s32Ret = HAL_I2C_Read(&pstI2cCtrl->stHalCtrl, pstI2cMsg->addr, pstI2cMsg->buf, (u32)pstI2cMsg->len);
        }
        else
        {
            dmsg_i2c_drvwarn("IN WRITE \n");
            s32Ret = HAL_I2C_Write(&pstI2cCtrl->stHalCtrl, pstI2cMsg->addr, pstI2cMsg->buf, (u32)pstI2cMsg->len);
        }
        if (pstI2cMsg->flags & 0x02)
            HAL_I2C_Release(&pstI2cCtrl->stHalCtrl);

        if (s32Ret)
        {
            break;
        }
    }
    HAL_I2C_Release(&pstI2cCtrl->stHalCtrl);

#ifdef CONFIG_CAM_CLK

#else
    clk_disable_unprepare(pstI2cClk);
    clk_put(pstI2cClk);
#endif

    if (s32Ret)
    {
        dmsg_i2c_drvwarn("i2c-%d xfer error\n", para_adapter->nr);
        para_num = 0;
    }
    else
    {
        dmsg_i2c_drvwarn("OK return xfer\n");
    }
    CamOsTsemUp(&pstI2cCtrl->stHalCtrl.stTsemID);
    return para_num;
}

static struct i2c_algorithm gsstr_i2c_algo = {
    .master_xfer   = sstar_i2c_master_xfer,
    .functionality = sstar_i2c_func,
};
static void drv_i2c_interrupt(u32 irq, void *para_platform_dev)
{
    s32            s32Ret;
    st_i2c_ctrl *  pstI2cCtrl;
    struct device *pstPdev;

    pstPdev    = para_platform_dev;
    pstI2cCtrl = pstPdev->driver_data;

    s32Ret = HAL_I2C_DmaTrDone(&pstI2cCtrl->stHalCtrl, true);
    if (s32Ret)
    {
        CamOsTsemUp(&(pstI2cCtrl->stHalCtrl.stTsemID));
    }
}
static s32 sstar_i2c_remove_srclk(struct platform_device *para_pdev)
{
    s32         s32Ret;
    u32         u32ParentNum;
    struct clk *pstI2cClk;
#ifdef CONFIG_CAM_CLK
    st_i2c_ctrl *pstI2Ctrl;
#endif

#ifdef CONFIG_CAM_CLK
    pstI2Ctrl = platform_get_drvdata(para_pdev);

    if (pstI2Ctrl->pvI2cClk)
    {
        CamClkSetOnOff(pstI2Ctrl->pvI2cClk, 0);
        CamClkUnregister(pstI2Ctrl->pvI2cClk);
    }
    return 0;
#else
    u32ParentNum = of_clk_get_parent_count(para_pdev->dev.of_node);
    if (u32ParentNum == 0)
    {
        dmsg_i2c_drverr("Fail to get parent count! Error Number : %d\n", u32ParentNum);
        s32Ret = -ENOENT;
        goto out;
    }

    pstI2cClk = of_clk_get(para_pdev->dev.of_node, 0);
    if (IS_ERR(pstI2cClk))
    {
        dmsg_i2c_drverr("of_clk_get err!\n");
        s32Ret = -ENOENT;
        goto out;
    }

    clk_disable_unprepare(pstI2cClk);
    clk_put(pstI2cClk);
    return 0;
#endif // endif CONFIG_CAM_CLK
out:
    return s32Ret;
}
/****************************************************/
//*func:
//*description:
//*parameter:
//*return:
/****************************************************/
static s32 sstar_i2c_probe(struct platform_device *para_pdev)
{
    st_i2c_ctrl *    pstI2cCtrl;
    u32              u32I2cSpeed;
    u32              u32I2cIrqNum;
    u32              u32I2cDmaEn;
    u32              u32I2cGoup;
    void __iomem *   pI2cBase;
    s32              s32Ret;
    struct resource *ret_res;

    u32I2cGoup = 0;

    ret_res = platform_get_resource(para_pdev, IORESOURCE_MEM, 0);
    if (!ret_res)
    {
        dmsg_i2c_drverr("get dev resource -ENOMEM\n");
        s32Ret = -ENOMEM;
        goto err_out;
    }

    pstI2cCtrl = devm_kzalloc(&(para_pdev->dev), sizeof(*pstI2cCtrl), GFP_KERNEL);
    if (!pstI2cCtrl)
    {
        dmsg_i2c_drverr("devm_kzalloc failed!\n");
        s32Ret = -ENOMEM;
        goto err_out;
    }

    s32Ret = CamofPropertyReadU32(para_pdev->dev.of_node, "i2c-group", &u32I2cGoup);
    if (s32Ret)
    {
        dmsg_i2c_drverr("get property i2c-group failed!\n");
        s32Ret = -ENOENT;
        goto err_out;
    }

    s32Ret = CamofPropertyReadU32(para_pdev->dev.of_node, "i2c-speed", &u32I2cSpeed);
    if (s32Ret)
    {
        dmsg_i2c_drverr("get property i2c-speed failed, group:%d!\n", u32I2cGoup);
        s32Ret = -ENOENT;
        goto err_out;
    }

    s32Ret = CamofPropertyReadU32(para_pdev->dev.of_node, "i2c-en-dma", &u32I2cDmaEn);
    if (s32Ret)
    {
        dmsg_i2c_drvwarn("get property i2c-en-dma failed, group:%d!\n", u32I2cGoup);
    }

    u32I2cIrqNum = CamIrqOfParseAndMap(para_pdev->dev.of_node, 0);
    if (u32I2cIrqNum == 0)
    {
        dmsg_i2c_drverr("can't find interrupts property, group:%d!\n", u32I2cGoup);
        s32Ret = -ENOENT;
        goto err_out;
    }

    if (!snprintf(pstI2cCtrl->u8IrqName, sizeof(pstI2cCtrl->u8IrqName), "i2c%d_Isr", u32I2cGoup))
    {
        dmsg_i2c_drverr("find irq reformat failed, group:%d!\n", u32I2cGoup);
        s32Ret = -ENOENT;
        goto err_out;
    }

    /*master mode and base addr*/
    pI2cBase                          = (void *)(IO_ADDRESS(ret_res->start));
    pstI2cCtrl->u32IrqNum             = u32I2cIrqNum;
    pstI2cCtrl->pDevice               = &(para_pdev->dev);
    pstI2cCtrl->stHalCtrl.u32EnDma    = u32I2cDmaEn;
    pstI2cCtrl->stHalCtrl.u32Group    = u32I2cGoup;
    pstI2cCtrl->stHalCtrl.u32Speed    = u32I2cSpeed;
    pstI2cCtrl->stHalCtrl.u64BankBase = (unsigned long)pI2cBase;
    dmsg_i2c_drvwarn("u64BankBase = 0x%llx\n", pstI2cCtrl->stHalCtrl.u64BankBase);
    pstI2cCtrl->stHalCtrl.calbak_i2c_set_srcclk = sstar_i2c_set_srclk;

    /*save i2c ctrl struct into device->driver_data*/
    platform_set_drvdata(para_pdev, pstI2cCtrl);

    /*i2c adapt device init&add*/
    pstI2cCtrl->stAdapter.owner = THIS_MODULE;
    pstI2cCtrl->stAdapter.class = I2C_CLASS_DEPRECATED;
    s32Ret =
        scnprintf(pstI2cCtrl->stAdapter.name, sizeof(pstI2cCtrl->stAdapter.name), "Sstar I2C adapter %d", u32I2cGoup);
    pstI2cCtrl->stAdapter.algo        = &gsstr_i2c_algo;
    pstI2cCtrl->stAdapter.dev.parent  = &(para_pdev->dev);
    pstI2cCtrl->stAdapter.nr          = u32I2cGoup;
    pstI2cCtrl->stAdapter.dev.of_node = para_pdev->dev.of_node;

    s32Ret = CamOsIrqRequest(u32I2cIrqNum, drv_i2c_interrupt, pstI2cCtrl->u8IrqName, (void *)&para_pdev->dev);
    if (s32Ret == 0)
    {
        dmsg_i2c_drvwarn("%d registered\n", u32I2cIrqNum);
    }
    else
    {
        dmsg_i2c_drverr("%d register failed\n", u32I2cIrqNum);
        s32Ret = -ENOENT;
        goto err_irq;
    }

    s32Ret = sstar_i2c_init(pstI2cCtrl, para_pdev);
    i2c_set_adapdata(&pstI2cCtrl->stAdapter, pstI2cCtrl);
    s32Ret = i2c_add_numbered_adapter(&pstI2cCtrl->stAdapter);
    if (s32Ret)
    {
        dmsg_i2c_drverr("add adapter err,group : %d\n", u32I2cGoup);
        goto err_adap;
    }

    return 0;
err_adap:
    i2c_del_adapter(&pstI2cCtrl->stAdapter);
err_irq:
    CamOsIrqFree(u32I2cIrqNum, (void *)&para_pdev->dev);
err_out:
    return s32Ret;
}

/****************************************************/
//*func:
//*description:
//*parameter:
//*return:
/****************************************************/
static s32 sstar_i2c_remove(struct platform_device *para_pdev)
{
    st_i2c_ctrl *pstI2cCtrl;
    s32          s32Ret = 0;

    pstI2cCtrl = platform_get_drvdata(para_pdev);
    s32Ret |= sstar_i2c_deinit(pstI2cCtrl);
    CamOsIrqFree(pstI2cCtrl->u32IrqNum, (void *)&para_pdev->dev);
    s32Ret |= sstar_i2c_remove_srclk(para_pdev);

    return s32Ret;
}

/****************************************************/
//*func:
//*description:
//*parameter:
//*return:
/****************************************************/
#ifdef CONFIG_PM_SLEEP
static s32 sstar_i2c_suspend(struct platform_device *para_pdev)
{
    st_i2c_ctrl *pstI2cCtrl;

    pstI2cCtrl = platform_get_drvdata(para_pdev);

    CamOsIrqFree(pstI2cCtrl->u32IrqNum, (void *)&para_pdev->dev);
    CamOsTsemDeinit(&pstI2cCtrl->stHalCtrl.bDmaDetctMod);
    sstar_i2c_remove_srclk(para_pdev);

    return 0;
}

/****************************************************/
//*func:
//*description:
//*parameter:
//*return:
/****************************************************/
static s32 sstar_i2c_resume(struct platform_device *para_pdev)
{
    st_i2c_ctrl *pstI2cCtrl;
    s32          s32Ret;
    u8 char      au8IrqName[20];

    pstI2cCtrl = platform_get_drvdata(para_pdev);

    s32Ret =
        CamOsIrqRequest(pstI2cCtrl->u32IrqNum, drv_i2c_interrupt, pstI2cCtrl->stAdapter.name, (void *)&para_pdev->dev);
    if (s32Ret == 0)
    {
        dmsg_i2c_drvwarn("%d registered\n", pstI2cCtrl->u32IrqNum);
    }
    else
    {
        dmsg_i2c_drverr("%d register failed\n", pstI2cCtrl->u32IrqNum);
        s32Ret = -ENOENT;
        goto err_irq;
    }

    s32Ret = sstar_i2c_init(pstI2cCtrl, para_pdev);

    return s32Ret;

err_irq:
    CamOsIrqFree(pstI2cCtrl->u32IrqNum, (void *)&para_pdev->dev);
    return s32Ret;
}

#endif // end #ifdef CONFIG_PM_SLEEP

static const struct of_device_id gsstrar_i2c_of_match[] = {
    {.compatible = "sstar,i2c", 0},
    {},
};
MODULE_DEVICE_TABLE(of, gsstrar_i2c_of_match);

static struct platform_driver gsstr_i2c_driver = {
    .probe  = sstar_i2c_probe,
    .remove = sstar_i2c_remove,
#ifdef CONFIG_PM_SLEEP
    .suspend = sstar_i2c_suspend,
    .resume  = sstar_i2c_resume,
#endif
    .driver =
        {
            .name           = "sstar,i2c",
            .owner          = THIS_MODULE,
            .of_match_table = gsstrar_i2c_of_match,
        },
};

static s32 __init sstar_i2c_init_driver(void)
{
    dmsg_i2c_drvwarn("init driver\n");
    return CamPlatformDriverRegister(&gsstr_i2c_driver);
}

static void __exit sstar_i2c_exit_driver(void)
{
    CamPlatformDriverRegister(&gsstr_i2c_driver);
}
module_init(sstar_i2c_init_driver);
module_exit(sstar_i2c_exit_driver);

MODULE_DESCRIPTION("Sstar I2C Bus Controller driver");
MODULE_AUTHOR("SSTAR");
MODULE_LICENSE("GPL v2");
