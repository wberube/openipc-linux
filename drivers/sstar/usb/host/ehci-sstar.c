/*
 * ehci-sstar.c- Sigmastar
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
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include "../../../usb/host/ehci.h"
#include "ehci-sstar.h"
#include "phy-sstar-u2phy.h"

extern unsigned long lx_mem_size;

static struct hc_driver __read_mostly ehci_sstar_hc_driver;

void ehci_hcd_sstar_miu_select(struct regmap *usbc)
{
    // void __iomem *usbc_base = (void __iomem *)hcd->usbc_base;

    printk("[USB] config miu select [%02x] [%02x] [%02x] [%02x]\n", USB_MIU_SEL0, USB_MIU_SEL1, USB_MIU_SEL2,
           USB_MIU_SEL3);

    /* [3:0]  : Lower bound of miu sel0 */
    /* [7:4]  : Upper bound of miu sel0 */
    regmap_write(usbc, (0x0A << 2), USB_MIU_SEL0);

    /* [3:0]  : Lower bound of miu sel1 */
    /* [7:4]  : Upper bound of miu sel1 */
    /* [11:8] : Lower bound of miu sel2 */
    /* [15:12]: Upper bound of miu sel2 */
    regmap_write(usbc, (0x0B << 2), (USB_MIU_SEL2 << 8) | USB_MIU_SEL1);

    /* [3:0] : Lower bound of miu sel3 */
    /* [7:4] : Upper bound of miu sel3 */
    /* [8]   : Enable miu partition mechanism */
    regmap_update_bits(usbc, (0x0C << 2), 0xFF, USB_MIU_SEL3);
    regmap_set_bits(usbc, (0x0C << 2), BIT(8));

    printk("[USB] enable miu lower bound address subtraction\n");
    regmap_set_bits(usbc, (0x07 << 2), BIT(8));
}

void ehci_hcd_sstar_usbc_settings(struct usb_hcd *hcd)
{
    struct regmap *usbc = (struct regmap *)hcd->usbc_base;

    if (!usbc)
        return;

    /* Enable use eof2 to reset state machine (power noise) */
    regmap_set_bits(usbc, (0x01 << 2), BIT(6));

    /* HS connection fail problem (Gate into VFALL state) */
    regmap_set_bits(usbc, (0x08 << 2), BIT(9));

    /* ENABLE_PV2MI_BRIDGE_ECO */
    regmap_set_bits(usbc, (0x05 << 2), BIT(6));

    /* _USB_MINI_PV2MI_BURST_SIZE */
    regmap_clear_bits(usbc, (0x05 << 2), BIT(12) | BIT(11) | BIT(10) | BIT(9));

    /* [11]: reg_preamble_en, to enable Faraday Preamble */
    regmap_set_bits(usbc, (0x07 << 2), BIT(11));

    /* [0]: reg_preamble_babble_fix, to patch Babble occurs in Preamble */
    /* [1]: reg_preamble_fs_within_pre_en, to patch FS crash problem */
    regmap_set_bits(usbc, (0x08 << 2), BIT(1) | BIT(0));

    /* Enable HS ISO IN Camera Cornor case ECO function */
    regmap_set_bits(usbc, (0x09 << 2), BIT(8));

    /* [0]: UHC speed type report should be reset by device disconnection */
    /* [1]: Port Change Detect (PCD) is triggered by babble. Pulse trigger will not hang this condition. */
    /* [2]: ENABLE_HC_RESET_FAIL_ECO, generation of hhc_reset_u */
    regmap_set_bits(usbc, (0x10 << 2), BIT(2) | BIT(1) | BIT(0));

    /* Use 2 SOFs to check disconnection */
    regmap_update_bits(usbc, (0x01 << 2), 0xFF00, BIT(10) | BIT(8));
    dev_info(hcd->self.controller, "%s\n", __func__);
    return;
}
/**
 * usb_ehci_au1xxx_probe - initialize Au1xxx-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
int ehci_hcd_sstar_drv_probe(struct platform_device *dev)
{
    struct resource *   res;
    int                 retval = 0;
    int                 irq    = -1;
    struct usb_hcd *    hcd;
    struct ehci_hcd *   ehci;
    u64                 dma_mask;
    u32                 vbus_ctrl;
    struct device_node *node = dev->dev.of_node;
    struct device_node *temp_node;
    struct phy *        generic_phy = NULL;
    struct sstar_u2phy *priv        = NULL;

    if (usb_disabled())
        return -ENODEV;

#if defined(CONFIG_OF)
    if (!dev->dev.platform_data)
    {
        dev_warn(&dev->dev, "[USB] no platform_data, device tree coming\n");
    }

    if (!dev->dev.dma_mask)
        dev->dev.dma_mask = &dev->dev.coherent_dma_mask;

    if (IS_ENABLED(CONFIG_ARM64) && IS_ENABLED(CONFIG_ZONE_DMA))
    {
#if defined(EHC_DMA_BIT_MASK)
        dma_mask = EHC_DMA_BIT_MASK;
#else
        /* default: 32bit to mask lowest 4G address */
        dma_mask = DMA_BIT_MASK(32);
#endif
    }
    else
        dma_mask = DMA_BIT_MASK(64);

    if (dma_set_mask(&dev->dev, dma_mask) || dma_set_coherent_mask(&dev->dev, dma_mask))
    {
        dev_err(&dev->dev, "[USB][EHC] cannot accept dma mask 0x%llx\n", dma_mask);
        return -EOPNOTSUPP;
    }

    dma_direct_set_offset(&dev->dev, MIU0_BASE, 0, lx_mem_size);

    dev_info(&dev->dev, "[USB][EHC] dma coherent_mask 0x%llx mask 0x%llx\n", dev->dev.coherent_dma_mask,
             *dev->dev.dma_mask);

    /* try to get irq from device tree */
    irq = irq_of_parse_and_map(dev->dev.of_node, 0);
#else
    if (dev->resource[2].flags != IORESOURCE_IRQ)
    {
        dev_warn(&dev->dev, "resource[2] is not IORESOURCE_IRQ");
    }
    else
    {
        irq = dev->resource[2].start;
    }
#endif

#if !defined(ENABLE_IRQ_REMAP)
    if (irq <= 0)
    {
        dev_err(&dev->dev, "[USB] can not get irq for %s\n", dev->name);
        return -ENODEV;
    }
#endif

    hcd = usb_create_hcd(&ehci_sstar_hc_driver, &dev->dev, "sstar");
    if (!hcd)
        return -ENOMEM;

    // res             = platform_get_resource(dev, IORESOURCE_MEM, 0);
    res             = platform_get_resource_byname(dev, IORESOURCE_MEM, "ehc_base");
    hcd->rsrc_start = res->start;
    hcd->rsrc_len   = resource_size(res);
    hcd->regs       = devm_ioremap_resource(&dev->dev, res);
    if (IS_ERR(hcd->regs))
    {
        dev_err(&dev->dev, "ioremap failed");
        retval = -ENOMEM;
        goto err1;
    }
    hcd->ehc_base = (uintptr_t)hcd->regs;

    if (!of_property_read_u32(node, "power-enable-pad", &vbus_ctrl))
    {
        dev_info(&dev->dev, "Get power-enable-pad from DTS GPIO(%d)\n", vbus_ctrl);
        retval = gpio_request(vbus_ctrl, "VBUS Ctrl Pin");
        if (retval < 0)
        {
            dev_err(&dev->dev, "Failed to request USB0-power-enable GPIO(%d)\n", vbus_ctrl);
        }
        else
        {
            gpio_direction_output(vbus_ctrl, 1);
        }
    }

    temp_node = of_parse_phandle(node, "syscon-utmi", 0);
    if (temp_node)
    {
        hcd->utmi_base = (uintptr_t)syscon_node_to_regmap(temp_node);
        if (IS_ERR((void *)hcd->utmi_base))
        {
            dev_err(&dev->dev, "failed to find utmi syscon regmap\n");
            return PTR_ERR((void *)hcd->utmi_base);
        }
        of_node_put(temp_node);
    }

    temp_node = of_parse_phandle(node, "syscon-bc", 0);
    if (temp_node)
    {
        hcd->bc_base = (uintptr_t)syscon_node_to_regmap(temp_node);
        if (IS_ERR((void *)hcd->bc_base))
        {
            dev_err(&dev->dev, "failed to find bc syscon regmap\n");
            return PTR_ERR((void *)hcd->bc_base);
        }
        of_node_put(temp_node);
    }

    temp_node = of_parse_phandle(node, "syscon-usbc", 0);
    if (temp_node)
    {
        hcd->usbc_base = (uintptr_t)syscon_node_to_regmap(temp_node);
        if (IS_ERR((void *)hcd->usbc_base))
        {
            dev_err(&dev->dev, "failed to find usbc syscon regmap\n");
            return PTR_ERR((void *)hcd->usbc_base);
        }
        of_node_put(temp_node);
    }

    ehci_hcd_sstar_miu_select((struct regmap *)hcd->usbc_base);
    hcd->has_tt = 1;

    ehci       = hcd_to_ehci(hcd);
    ehci->caps = hcd->regs;
    ehci->regs =
        (struct ehci_regs *)((uintptr_t)hcd->regs + HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase)));

    dev_info(&dev->dev, "[USB] hcd ehc:%08lx usbc:%08lx bc:%08lx\n", hcd->ehc_base, hcd->usbc_base, hcd->bc_base);

    /* cache this readonly data; minimize chip reads */
    ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

    dev_info(&dev->dev, "[USB] %s irq --> %d\n", dev->name, irq);

    /* IRQF_DISABLED was removed from kernel 4.1
       commit d8bf368d0631d4bc2612d8bf2e4e8e74e620d0cc. */
    retval = usb_add_hcd(hcd, irq, 0);

    ehci_hcd_sstar_usbc_settings(hcd);

    hcd->root_port_devnum  = 0;
    hcd->enum_port_flag    = 0;
    hcd->enum_dbreset_flag = 0;
    hcd->lock_usbreset     = __SPIN_LOCK_UNLOCKED(hcd->lock_usbreset);

    if (retval != 0)
        goto err1;

    generic_phy = devm_of_phy_get_by_index(&dev->dev, node, 0);
    if (generic_phy)
    {
        priv = phy_get_drvdata(generic_phy);
        if (priv)
        {
            hcd->usb_phy = &priv->usb_phy;
        }
    }

    return retval;

err1:
    usb_put_hcd(hcd);
    return retval;
}
/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_ehci_hcd_au1xxx_remove - shutdown processing for Au1xxx-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_ehci_hcd_au1xxx_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static int ehci_hcd_sstar_drv_remove(struct platform_device *dev)
{
    struct usb_hcd *hcd = platform_get_drvdata(dev);

    usb_remove_hcd(hcd);
    usb_put_hcd(hcd);
    return 0;
}

#ifdef CONFIG_PM
static int ehci_hcd_sstar_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct usb_hcd *hcd = platform_get_drvdata(pdev);

    printk("ehci_hcd_sstar_platform_suspend...port %d\n", hcd->port_index);
    return ehci_suspend(hcd, false);
}

static int ehci_hcd_sstar_drv_resume(struct platform_device *pdev)
{
    struct usb_hcd *hcd = platform_get_drvdata(pdev);

    printk("ehci_hcd_sstar_platform_resume...port %d\n", hcd->port_index);

    ehci_resume(hcd, false);
    return 0;
}
#endif

static struct of_device_id sstar_ehci_of_device_ids[] = {
    {.compatible = "Sstar-ehci-1"}, {.compatible = "Sstar-ehci-2"}, {.compatible = "Sstar-ehci-3"},
    {.compatible = "Sstar-ehci-4"}, {.compatible = "Sstar-ehci-5"}, {},
};

static struct platform_driver ehci_hcd_sstar_driver = {.probe  = ehci_hcd_sstar_drv_probe,
                                                       .remove = ehci_hcd_sstar_drv_remove,
#ifdef CONFIG_PM
                                                       .suspend = ehci_hcd_sstar_drv_suspend,
                                                       .resume  = ehci_hcd_sstar_drv_resume,
#endif
                                                       .driver = {
                                                           .name = "Sstar-ehci",
#if defined(CONFIG_OF)
                                                           .of_match_table = sstar_ehci_of_device_ids,
#endif
                                                           //      .bus    = &platform_bus_type,
                                                       }};
