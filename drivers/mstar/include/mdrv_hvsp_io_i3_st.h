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

/**
 *  @file mdrv_hvsp_io_st.h
 *  @brief hvsp Driver IOCTL parameter interface
 */

 /**
 * \ingroup hvsp_group
 * @{
 */

#ifndef _MDRV_HVSP_IO_ST_H
#define _MDRV_HVSP_IO_ST_H

//=============================================================================
// Defines
//=============================================================================
#define IOCTL_HVSP_VERSION                        0x0100


//=============================================================================
// enum
//=============================================================================
/**
* Used to setup the input source of hvsp device
*/
typedef enum
{
    E_IOCTL_HVSP_SRC_ISP,       ///< input source: ISP
    E_IOCTL_HVSP_SRC_BT656,     ///< input source: BT656
    E_IOCTL_HVSP_SRC_DRAM,      ///< input source: DRAM
    E_IOCTL_HVSP_SRC_HVSP,      ///< input source: HVSP1
    E_IOCTL_HVSP_SRC_PAT_TGEN,  ///< input source: PATGEN
    E_IOCTL_HVSP_SRC_NUM,       ///< The max number of input source
}EN_IOCTL_HVSP_SRC_TYPE;

/**
* Used to setup the color format of hvsp device
*/
typedef enum
{
    E_IOCTL_HVSP_COLOR_RGB,     ///< color format:RGB
    E_IOCTL_HVSP_COLOR_YUV444,  ///< color format:YUV444
    E_IOCTL_HVSP_COLOR_YUV422,  ///< color format:YUV422
    E_IOCTL_HVSP_COLOR_YUV420,  ///< color format:YUV420
    E_IOCTL_HVSP_COLOR_NUM,     ///< The max number of color format
}EN_IOCTL_HVSP_COLOR_TYPE;

/**
* Used to setup the IPR/W status of hvsp device
*/
typedef enum
{
    E_IOCTL_HVSP_MCNR_YCM_R  = 0x1,    ///< IP only read
    E_IOCTL_HVSP_MCNR_YCM_W  = 0x2,    ///< IP only write
    E_IOCTL_HVSP_MCNR_YCM_RW = 0x3,    ///< IP R/W
    E_IOCTL_HVSP_MCNR_CIIR_R = 0x4,    ///< IP only read
    E_IOCTL_HVSP_MCNR_CIIR_W = 0x8,    ///< IP only write
    E_IOCTL_HVSP_MCNR_CIIR_RW = 0xC,    ///< IP R/W
    E_IOCTL_HVSP_MCNR_NON = 0x10,    ///< IP none open
}EN_IOCTL_HVSP_MCNR_TYPE;

/**
* Used to setup the OSD locate of hvsp device
*/
typedef enum
{
    E_IOCTL_HVSP_OSD_LOC_AFTER  = 0,    ///< after hvsp
    E_IOCTL_HVSP_OSD_LOC_BEFORE = 1,    ///< before hvsp
}EN_IOCTL_HVSP_OSD_LOC_TYPE;

/**
* Used to setup the FB locate of hvsp device
*/
typedef enum
{
    EN_IOCTL_HVSP_FBMG_SET_LDCPATH_ON      = 0x1,
    EN_IOCTL_HVSP_FBMG_SET_LDCPATH_OFF     = 0x2,
    EN_IOCTL_HVSP_FBMG_SET_DNR_Read_ON     = 0x4,
    EN_IOCTL_HVSP_FBMG_SET_DNR_Read_OFF    = 0x8,
    EN_IOCTL_HVSP_FBMG_SET_DNR_Write_ON    = 0x10,
    EN_IOCTL_HVSP_FBMG_SET_DNR_Write_OFF   = 0x20,
    EN_IOCTL_HVSP_FBMG_SET_DNR_BUFFER_1    = 0x40,
    EN_IOCTL_HVSP_FBMG_SET_DNR_BUFFER_2    = 0x80,
    EN_IOCTL_HVSP_FBMG_SET_UNLOCK          = 0x100,
    EN_IOCTL_HVSP_FBMG_SET_DNR_COMDE_ON    = 0x200,
    EN_IOCTL_HVSP_FBMG_SET_DNR_COMDE_OFF   = 0x400,
    EN_IOCTL_HVSP_FBMG_SET_DNR_COMDE_265OFF   = 0x800,
    EN_IOCTL_HVSP_FBMG_SET_PRVCROP_ON      = 0x1000,
    EN_IOCTL_HVSP_FBMG_SET_PRVCROP_OFF     = 0x2000,
    EN_IOCTL_HVSP_FBMG_SET_CIIR_ON      = 0x4000,
    EN_IOCTL_HVSP_FBMG_SET_CIIR_OFF     = 0x8000,
}EN_IOCTL_HVSP_FBMG_SET_TYPE;
//=============================================================================
// struct
//=============================================================================
/**
* Used to get HVSP drvier version
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    unsigned int   u32Version;      ///< version
    unsigned int   VerChk_Size;     ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_VERSION_CONFIG;

/**
* Used to setup the crop size of hvsp device
*/
typedef struct
{
    unsigned short u16X;        ///< crop frame start x point
    unsigned short u16Y;        ///< crop frame start y point
    unsigned short u16Width;    ///< crop width size
    unsigned short u16Height;   ///< crop height size
}__attribute__ ((__packed__))ST_IOCTL_HVSP_WINDOW_CONFIG;

/**
* Used to setup the HVSP timing gen of hvsp device
*/
typedef struct
{
    unsigned char  bInterlace;      ///< is interlace or progressive
    unsigned short u16Htotal;       ///< Htt
    unsigned short u16Vtotal;       ///< Vtt
    unsigned short u16Vfrequency;   ///< Vfreq
}ST_IOCTL_HVSPTIMING_CONFIG;

//=============================================================================
// struct for IOCTL_HVSP_SET_IN_CONFIG
/**
* Used to setup the input mux ,input capture window of hvsp device
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    EN_IOCTL_HVSP_SRC_TYPE        enSrcType;    ///< Input source type
    EN_IOCTL_HVSP_COLOR_TYPE      enColor;      ///< color type
    ST_IOCTL_HVSP_WINDOW_CONFIG   stCaptureWin; ///< Input Source Size(input src is scl capture win
    ST_IOCTL_HVSPTIMING_CONFIG    stTimingCfg;  ///< Input Timing
    // VerChk_Version & VerChk_Size must be the latest 2 parameter and
    // the order can't be changed
    unsigned int   VerChk_Size; ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_INPUT_CONFIG;


// struct for IOCTL_HVSP_SET_OUT_CONFIG
/**
* Used to setup the output config of hvsp device
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    EN_IOCTL_HVSP_COLOR_TYPE      enColor;      ///< color type
    ST_IOCTL_HVSP_WINDOW_CONFIG   stDisplayWin; ///< display window size
    ST_IOCTL_HVSPTIMING_CONFIG    stTimingCfg;  ///< output timing
    unsigned int   VerChk_Size; ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_OUTPUT_CONFIG;
/**
* Used to set CLK mux of hvsp device
*/
typedef struct
{
    struct clk* idclk;  ///< idclk
    struct clk* fclk1;  ///< fclk (SC1 SC2
    struct clk* fclk2;  ///< fclk (SC3
    struct clk* odclk;  ///< odclk (display Lpll
}ST_IOCTL_HVSP_CLK_CONFIG;
// sturct for IOCTL_HVSP_SET_SCALING_CONFIG
/**
* Used to setup the crop and HVSP123 scaling configuration of hvsp device
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    unsigned short              u16Src_Width;   ///< post crop IN width
    unsigned short              u16Src_Height;  ///< post crop IN height
    unsigned short              u16Dsp_Width;   ///< after HVSP width
    unsigned short              u16Dsp_Height;  ///< after HVSP height
    unsigned char               bCropEn;        ///< Is post crop En
    ST_IOCTL_HVSP_WINDOW_CONFIG stCropWin;      ///< post crop size
    // VerChk_Version & VerChk_Size must be the latest 2 parameter and
    // the order can't be changed
    unsigned int   VerChk_Size; ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_SCALING_CONFIG;

//IOCTL_HVSP_REQ_MEM_CONFIG
/**
* Used to allocate the buffer and setup framebuffer configuration of hvsp device
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    unsigned short              u16Vsize;   ///< framebuffer height
    unsigned short              u16Pitch;   ///< framebuffer width
    unsigned long               u32MemSize; ///< height*width *2(YUV422) *2(2frame)
    // VerChk_Version & VerChk_Size must be the latest 2 parameter and
    // the order can't be changed
    unsigned int   VerChk_Size; ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_REQ_MEM_CONFIG;

//IOCTL_HVSP_SET_MISC_CONFIG
/**
* Used to setup the register of hvsp device
*/
typedef struct
{
    unsigned char   u8Cmd;      ///< register value
    unsigned long   u32Size;    ///< number
    unsigned long   u32Addr;    ///< bank&addr
}__attribute__ ((__packed__))ST_IOCTL_HVSP_MISC_CONFIG;

//IOCTL HVSP_SET_POST_CROP_CONFIG
/**
* Used to setup the post crop of hvsp device if need
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    unsigned char  bCropEn;     ///< post crop En
    unsigned short u16X;        ///< crop frame start x point
    unsigned short u16Y;        ///< crop frame start y point
    unsigned short u16Width;    ///< crop frame width
    unsigned short u16Height;   ///< crop frame height
    unsigned char  bFmCntEn;    ///< Is use CMDQ to set
    unsigned char  u8FmCnt;     ///< when frame count
    // VerChk_Version & VerChk_Size must be the latest 2 parameter and
    // the order can't be changed
    unsigned int   VerChk_Size; ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_POSTCROP_CONFIG;

//IOCTL HVSP_SET_POST_CROP_CONFIG
/**
* Used to get resolution information of hvsp device
*/
typedef struct
{
    unsigned short u16X;            ///< x vs input src
    unsigned short u16Y;            ///< y vs input src
    unsigned short u16Width;        ///< display width
    unsigned short u16Height;       ///< display height
    unsigned short u16crop2inWidth; ///< framebuffer width
    unsigned short u16crop2inHeight;///< framebuffer height
    unsigned short u16crop2OutWidth; ///< after crop width
    unsigned short u16crop2OutHeight;///< after crop height
}ST_IOCTL_HVSP_SCINFORM_CONFIG;

//IOCTL_HVSP_GET_PRIVATE_ID_CONFIG
/**
* Used to get private ID of hvsp device
*/
typedef struct
{
    signed long s32Id;  ///< private ID
}ST_IOCTL_HVSP_PRIVATE_ID_CONFIG;

//IOCTL_HVSP_PRIMASK_CONFIG
/**
* Used to set MASK of hvsp device
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    unsigned char bMask; ///<Mask or not
    unsigned char u8idx; ///< mask id
    ST_IOCTL_HVSP_WINDOW_CONFIG stMaskWin;      ///< Mask info
    unsigned int   VerChk_Size; ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_PRIMASK_CONFIG;
/**
* Used to set MASK trigger of hvsp device
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    unsigned char bEn; ///<Mask enable
    unsigned int   VerChk_Size; ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_PRIMASK_TRIGGER_CONFIG;

//IOCTL_HVSP_SET_OSD_CONFIG
/**
* Used to set OSD of hvsp device
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    unsigned char              bEn;          ///< OSD enable
    EN_IOCTL_HVSP_OSD_LOC_TYPE enOSD_loc;    ///< OSD locate
    unsigned char              bOSDBypass;   ///< OSD bypass
    unsigned char              bWTMBypass;   ///<WTM bypass
    // VerChk_Version & VerChk_Size must be the latest 2 parameter and
    // the order can't be changed
    unsigned int   VerChk_Size; ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_OSD_CONFIG;

//IOCTL_HVSP_SET_FB_MANAGE_CONFIG
/**
* Used to set OSD of hvsp device
*/
typedef struct
{
    unsigned int   VerChk_Version ; ///< VerChk version
    EN_IOCTL_HVSP_FBMG_SET_TYPE enSet;
    // VerChk_Version & VerChk_Size must be the latest 2 parameter and
    // the order can't be changed
    unsigned int   VerChk_Size; ///< VerChk Size
}__attribute__ ((__packed__)) ST_IOCTL_HVSP_SET_FB_MANAGE_CONFIG;

//=============================================================================

//=============================================================================



#endif //
/** @} */ // end of hvsp_group
