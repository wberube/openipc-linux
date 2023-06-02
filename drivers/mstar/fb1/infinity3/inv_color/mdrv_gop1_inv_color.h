/******************************************************************************
 *-----------------------------------------------------------------------------
 *
 *    Copyright (c) 2011 MStar Semiconductor, Inc.  All rights reserved.
 *
 *-----------------------------------------------------------------------------
 * FILE NAME      mdrv_inv_color.h
 * DESCRIPTION
 *          Including some MACRO needed in msb250xfb.c
 *                    Defined CLRREG16(), SETREG16()
 *          (refer to include/asm-arm/arch-msb25xx/io.h)
 *                    Defined used MASK and setting value
 *                    Defined CHIPTOP, OP2, VOP, GOP Base Addres in Physical Address
 *                    and theirs offset    (drvop2.h and drvgop.h)
 *                    Declared Boot_splash array. the image content shown in boot
 *          (boot_splasy.h)
 *            refering sources list
 *                    drvop2.h and drvgop.h
 *              boot_splasy.h
 *
 * AUTHOR         Chun Fan
 *
 * HISTORY
 *                2008/05/06    Chun    initial version
 *                2008/10/02    Chun    add MSB251x code, add CONFIG_ARCH_MSB251X
 *                                      add some IP base address Macro
 *                                      BK_LPLL_BASE, BK_DAC_BASE,
 *                                      and OP2_DITH_REG_BASE
 *
 *        linux/drivers/video/msb250xfb.h    --    msb250xfb frame buffer device
 ******************************************************************************/

#include <asm/io.h>
#include <linux/delay.h>
#include<linux/slab.h>
#include "../include/ms_types.h"
#include "../include/ms_platform.h"

#define GET_REG8_ADDR_GOP1_invColor(x, y)     (x+(y)*2)
#define GET_REG16_ADDR_GOP1_invColor(x, y)    (x+(y)*4)



/* ========================================================================= */
/* Define HW base address */
#define BASE_REG_RIU_PA_GOP1_invColor           (0x1F000000)

//GOP Base register defines

//inverse color base address
#define mdrv_BASE_REG_GOP1_invColor  GET_REG16_ADDR_GOP1_invColor(BASE_REG_RIU_PA_GOP1_invColor, (0x121A00/2))

//inverse color engine update done flag address
#define mdrv_BASE_REG_GOP1_invColor_AE_Update_Done  GET_REG16_ADDR_GOP1_invColor(BASE_REG_RIU_PA_GOP1_invColor, (0x121800/2))
#define GOP1_invColor_REG_AE_Update_Done        BK_REG_GOP1_invColor(0x0A)
#define GOP1_invColor_AE_Update_Done_mask       0x0080

#define BK_REG_GOP1_invColor(reg)               ((reg) * 4)

//--------------------------------------------------------------------------------------------------
//  Defines
//--------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// inverse color settings for GOP1
//------------------------------------------------------------------------------

#define GOP1_invColor_REG_BASE_SETTINGS         BK_REG_GOP1_invColor(0x3E)
#define GOP1_invColor_enable_mask               0x0001 // 1:enable, 0:disable
#define GOP1_invColor_gop_output_mode           0x0002 // 1:YUV, 0:RGB
#define GOP1_invColor_update_mode               0x0004 // 1:by cpu, 0:by engine
#define GOP1_invColor_color_mode                0x0008 // 1.complementary color, 0:user define
#define GOP1_invColor_AE_Y_Threshold_mask       0xFF00

#define GOP1_invColor_REG_USER_COLOR_RG         BK_REG_GOP1_invColor(0x3F)
#define GOP1_invColor_user_color_R_mask         0x00FF
#define GOP1_invColor_user_color_G_mask         0xFF00

#define GOP1_invColor_REG_USER_COLOR_B          BK_REG_GOP1_invColor(0x44)
#define GOP1_invColor_user_color_B_mask         0x00FF

#define GOP1_invColor_REG_AE_X_BLK_NUM          BK_REG_GOP1_invColor(0x44)
#define GOP1_invColor_AE_X_BLK_NUM_mask         0x7F00

#define GOP1_invColor_REG_X_PIX_OFFSET          BK_REG_GOP1_invColor(0x45)

#define GOP1_invColor_REG_Y_PIX_OFFSET          BK_REG_GOP1_invColor(0x46)

#define GOP1_invColor_REG_BLK_INDEX_OFFSET      BK_REG_GOP1_invColor(0x47)

#define GOP1_invColor_REG_BLK_WIDTH             BK_REG_GOP1_invColor(0x49)

#define GOP1_invColor_REG_BLK_HEIGHT            BK_REG_GOP1_invColor(0x4A)

#define GOP1_invColor_REG_SRAM_SETTINGS         BK_REG_GOP1_invColor(0x4B)
#define GOP1_invColor_SRAM_ADDR_mask            0x01FF
#define GOP1_invColor_SRAM_WRITE_enable         0x0200
#define GOP1_invColor_SRAM_READ_enable          0x0400
#define GOP1_invColor_SRAM_CPU_UPDATE_DONE      0x0800

#define GOP1_invColor_REG_SRAM_WRITE_DATA1      BK_REG_GOP1_invColor(0x4C)

#define GOP1_invColor_REG_SRAM_WRITE_DATA2      BK_REG_GOP1_invColor(0x4D)

#define GOP1_invColor_REG_SRAM_READ_DATA1       BK_REG_GOP1_invColor(0x4E)

#define GOP1_invColor_REG_SRAM_READ_DATA2       BK_REG_GOP1_invColor(0x4F)


#define GOP1_invColor_DEBUG 0
#if (GOP1_invColor_DEBUG==1)
#define GOP1_invColor_DBG(fmt, arg...) printk(KERN_INFO fmt, ##arg)
#else
#define GOP1_invColor_DBG(fmt, arg...)
#endif

//global functions
void GOP1_invColor_init(void);
void GOP1_invColor_Enable(unsigned char bEn);
void GOP1_invColor_Set_UpdateMode(unsigned char bMode);
void GOP1_invColor_Set_Y_Threshold(int AE_Y_Thres);
void GOP1_invColor_Set_AE_Config(int AE_Blk_Width, int AE_Blk_Height, int AE_x_res);
void GOP1_invColor_Set_AE_Config_Scaling(int AE_Blk_Width, int AE_Blk_Height, int AE_x_blk_num);
void GOP1_invColor_Set_Crop_Config(int crop_x_cor, int crop_y_cor, int AE_Blk_Width, int AE_Blk_Height, int AE_x_res);
int GOP1_invColor_CPU_Upate_InvTable(int AE_Blk_Width, int AE_Blk_Height, int AE_x_res, int AE_y_res);
void GOP1_invColor_Debug_Mode(void);
void GOP1_invColor_DebugMode_UpdateInvTable(void);