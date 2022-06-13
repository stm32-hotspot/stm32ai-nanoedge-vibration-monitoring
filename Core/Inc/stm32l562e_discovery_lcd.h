/**
  ******************************************************************************
  * @file    stm32l562e_discovery_lcd.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l562e_discovery_lcd.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32L562E_DISCOVERY_LCD_H
#define STM32L562E_DISCOVERY_LCD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_sram.h"
#include "stm32l562e_discovery_errno.h"
#include "../Fonts/fonts.h"
#include "st7789h2.h"

/** @defgroup STM32L562E-DK_LCD_Exported_Constants STM32L562E-DK LCD Exported Constants
  * @{
  */
/* LCD instances */
#define LCD_INSTANCES_NBR 1U

/* LCD orientations */
#define LCD_ORIENTATION_PORTRAIT          0U
#define LCD_ORIENTATION_LANDSCAPE         1U
#define LCD_ORIENTATION_PORTRAIT_ROT180   2U
#define LCD_ORIENTATION_LANDSCAPE_ROT180  3U

/* LCD colors */
#define LCD_COLOR_BLUE          0xFF0000FFUL
#define LCD_COLOR_GREEN         0xFF00FF00UL
#define LCD_COLOR_RED           0xFFFF0000UL
#define LCD_COLOR_CYAN          0xFF00FFFFUL
#define LCD_COLOR_MAGENTA       0xFFFF00FFUL
#define LCD_COLOR_YELLOW        0xFFFFFF00UL
#define LCD_COLOR_LIGHTBLUE     0xFF8080FFUL
#define LCD_COLOR_LIGHTGREEN    0xFF80FF80UL
#define LCD_COLOR_LIGHTRED      0xFFFF8080UL
#define LCD_COLOR_LIGHTCYAN     0xFF80FFFFUL
#define LCD_COLOR_LIGHTMAGENTA  0xFFFF80FFUL
#define LCD_COLOR_LIGHTYELLOW   0xFFFFFF80UL
#define LCD_COLOR_DARKBLUE      0xFF000080UL
#define LCD_COLOR_DARKGREEN     0xFF008000UL
#define LCD_COLOR_DARKRED       0xFF800000UL
#define LCD_COLOR_DARKCYAN      0xFF008080UL
#define LCD_COLOR_DARKMAGENTA   0xFF800080UL
#define LCD_COLOR_DARKYELLOW    0xFF808000UL
#define LCD_COLOR_WHITE         0xFFFFFFFFUL
#define LCD_COLOR_LIGHTGRAY     0xFFD3D3D3UL
#define LCD_COLOR_GRAY          0xFF808080UL
#define LCD_COLOR_DARKGRAY      0xFF404040UL
#define LCD_COLOR_BLACK         0xFF000000UL
#define LCD_COLOR_BROWN         0xFFA52A2AUL
#define LCD_COLOR_ORANGE        0xFFFF9000UL	// 0xFFFFA500UL

/* Defintion of Official ST Colors */
#define LCD_COLOR_ST_BLUE_DARK   0xFF002052UL
#define LCD_COLOR_ST_BLUE        0xFF39A9DCUL
#define LCD_COLOR_ST_BLUE_LIGHT  0xFFD1E4F3UL
#define LCD_COLOR_ST_GREEN_LIGHT 0xFFBBCC01UL
#define LCD_COLOR_ST_GREEN_DARK  0xFF003D14UL
#define LCD_COLOR_ST_YELLOW      0xFFFFF000UL	// 0xFFFFD300UL
#define LCD_COLOR_ST_BROWN       0xFF5C0915UL
#define LCD_COLOR_ST_PINK        0xFFD4007AUL
#define LCD_COLOR_ST_PURPLE      0xFF590D58UL
#define LCD_COLOR_ST_GRAY_DARK   0xFF4F5251UL
#define LCD_COLOR_ST_GRAY        0xFF90989EUL
#define LCD_COLOR_ST_GRAY_LIGHT  0xFFB9C4CAUL

/* LCD_Exported_Constants LCD Exported Constants */
#define LCD_PIXEL_FORMAT_ARGB8888        0x00000000U   /*!< ARGB8888 LTDC pixel format */
#define LCD_PIXEL_FORMAT_RGB888          0x00000001U   /*!< RGB888 LTDC pixel format   */
#define LCD_PIXEL_FORMAT_RGB565          0x00000002U   /*!< RGB565 LTDC pixel format   */
#define LCD_PIXEL_FORMAT_ARGB1555        0x00000003U   /*!< ARGB1555 LTDC pixel format */
#define LCD_PIXEL_FORMAT_ARGB4444        0x00000004U   /*!< ARGB4444 LTDC pixel format */
#define LCD_PIXEL_FORMAT_L8              0x00000005U   /*!< L8 LTDC pixel format       */
#define LCD_PIXEL_FORMAT_AL44            0x00000006U   /*!< AL44 LTDC pixel format     */
#define LCD_PIXEL_FORMAT_AL88            0x00000007U   /*!< AL88 LTDC pixel format     */


/** @defgroup STM32L562E-DK_LCD_Exported_Types STM32L562E-DK LCD Exported Types
  * @{
  */
#if (USE_HAL_SRAM_REGISTER_CALLBACKS == 1)
typedef struct
{
  pSRAM_CallbackTypeDef  pMspFmcInitCb;
  pSRAM_CallbackTypeDef  pMspFmcDeInitCb;
} BSP_LCD_Cb_t;
#endif /* (USE_HAL_SRAM_REGISTER_CALLBACKS == 1) */


/** @defgroup BSP_LCD_Exported_Types STM32 LCD Utility Exported Types
  * @{
  */
#define LINE(x)			((x) * (((sFONT *)BSP_LCD_GetFont())->Height))
#define ABS(X)			((X) > 0 ? (X) : -(X))
#define POLY_X(Z)		((int32_t)((Points + (Z))->X))
#define POLY_Y(Z)		((int32_t)((Points + (Z))->Y))

#define CONVERTARGB88882RGB565(Color)((((Color & 0xFFU) >> 3) & 0x1FU) |\
                                     (((((Color & 0xFF00U) >> 8) >>2) & 0x3FU) << 5) |\
                                     (((((Color & 0xFF0000U) >> 16) >>3) & 0x1FU) << 11))

#define CONVERTRGB5652ARGB8888(Color)(((((((Color >> 11) & 0x1FU) * 527) + 23) >> 6) << 16) |\
                                     ((((((Color >> 5) & 0x3FU) * 259) + 33) >> 6) << 8) |\
                                     ((((Color & 0x1FU) * 527) + 23) >> 6) | 0xFF000000)

/**
  * @brief  LCD Utility Drawing main properties
  */
typedef struct
{
  uint32_t  TextColor; /*!< Specifies the color of text */
  uint32_t  BackColor; /*!< Specifies the background color below the text */
  sFONT    *pFont;     /*!< Specifies the font used for the text */
  uint32_t  LcdLayer;
  uint32_t  LcdDevice;
  uint32_t  LcdXsize;
  uint32_t  LcdYsize;
  uint32_t  LcdPixelFormat;
} BSP_LCD_Ctx_t;

/** @defgroup LCD_Driver_structure  LCD Driver structure
  * @{
  */
typedef struct
{
  /* Control functions */
  int32_t (*Init             )(void*, uint32_t, uint32_t);
  int32_t (*DeInit           )(void*);
  int32_t (*ReadID           )(void*, uint32_t*);
  int32_t (*DisplayOn        )(void*);
  int32_t (*DisplayOff       )(void*);
  int32_t (*SetBrightness    )(void*, uint32_t);
  int32_t (*GetBrightness    )(void*, uint32_t*);
  int32_t (*SetOrientation   )(void*, uint32_t);
  int32_t (*GetOrientation   )(void*, uint32_t*);

  /* Drawing functions*/
  int32_t ( *SetCursor       ) (void*, uint32_t, uint32_t);
  int32_t ( *DrawBitmap      ) (void*, uint32_t, uint32_t, uint8_t *);
  int32_t ( *FillRGBRect     ) (void*, uint32_t, uint32_t, uint8_t*, uint32_t, uint32_t);
  int32_t ( *DrawHLine       ) (void*, uint32_t, uint32_t, uint32_t, uint32_t);
  int32_t ( *DrawVLine       ) (void*, uint32_t, uint32_t, uint32_t, uint32_t);
  int32_t ( *FillRect        ) (void*, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
  int32_t ( *GetPixel        ) (void*, uint32_t, uint32_t, uint32_t*);
  int32_t ( *SetPixel        ) (void*, uint32_t, uint32_t, uint32_t);
  int32_t ( *GetXSize        ) (void*, uint32_t *);
  int32_t ( *GetYSize        ) (void*, uint32_t *);
} LCD_Drv_t;

/**
  * @brief  LCD Utility drawing Line alignment mode definitions
  */
typedef enum
{
  CENTER_MODE             = 0x01,    /*!< Center mode */
  RIGHT_MODE              = 0x02,    /*!< Right mode  */
  LEFT_MODE               = 0x03     /*!< Left mode   */
} Text_AlignModeTypdef;

/**
  * @brief  LCD Utility Drawing point (pixel) geometric definition
  */
typedef struct
{
  int16_t X; /*!< geometric X position of drawing */
  int16_t Y; /*!< geometric Y position of drawing */
} Point;

/**
  * @brief  Pointer on LCD Utility Drawing point (pixel) geometric definition
  */
typedef Point * pPoint;



/** @defgroup STM32L562E-DK_LCD_Exported_Variables STM32L562E-DK LCD Exported Variables
  * @{
  */
extern SRAM_HandleTypeDef hlcd_sram[LCD_INSTANCES_NBR];
extern void      *Lcd_CompObj[LCD_INSTANCES_NBR];
extern LCD_Drv_t *Lcd_Drv[LCD_INSTANCES_NBR];

/** @defgroup STM32L562E-DK_LCD_Exported_Functions STM32L562E-DK LCD Exported Functions
  * @{
  */
int32_t  BSP_LCD_Init(uint32_t Instance, uint32_t Orientation);
int32_t  BSP_LCD_DeInit(uint32_t Instance);
int32_t  BSP_LCD_DisplayOn(uint32_t Instance);
int32_t  BSP_LCD_DisplayOff(uint32_t Instance);
int32_t  BSP_LCD_SetBrightness(uint32_t Instance, uint32_t Brightness);
int32_t  BSP_LCD_GetBrightness(uint32_t Instance, uint32_t *Brightness);
int32_t  BSP_LCD_GetXSize(uint32_t Instance, uint32_t *Xsize);
int32_t  BSP_LCD_GetYSize(uint32_t Instance, uint32_t *Ysize);
int32_t  BSP_LCD_GetFormat(uint32_t Instance, uint32_t *Format);

int32_t  BSP_LCD_SetFuncDriver(void);
void     BSP_LCD_SetTextColor(uint32_t Color);
uint32_t BSP_LCD_GetTextColor(void);
void     BSP_LCD_SetBackColor(uint32_t Color);
uint32_t BSP_LCD_GetBackColor(void);
void     BSP_LCD_SetFont(sFONT *fonts);
sFONT*	 BSP_LCD_GetFont(void);
int32_t  BSP_LCD_Clear(uint32_t Instance, uint32_t Color);
int32_t  BSP_LCD_ClearStringLine(uint32_t Instance, uint32_t Line);
int32_t BSP_LCD_DisplayStringAtLine(uint32_t Instance, uint32_t Line, uint8_t *ptr, Text_AlignModeTypdef Mode);
int32_t  BSP_LCD_DisplayStringAt(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode);
int32_t  BSP_LCD_DisplayChar(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t Ascii);
int32_t	 BSP_LCD_DrawChar(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, const uint8_t *pData);

int32_t  BSP_LCD_ReadPixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t *Color);
int32_t  BSP_LCD_WritePixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Color);
int32_t	 BSP_LCD_DrawLine(uint32_t Instance, uint32_t Xpos1, uint32_t Ypos1, uint32_t Xpos2, uint32_t Ypos2, uint32_t Color);
int32_t  BSP_LCD_DrawHLine(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Length, uint32_t Color);
int32_t  BSP_LCD_DrawVLine(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Length, uint32_t Color);
int32_t  BSP_LCD_DrawRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height, uint32_t Color);
int32_t  BSP_LCD_DrawCircle(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Radius, uint32_t Color);
int32_t  BSP_LCD_DrawPolygon(uint32_t Instance, pPoint Points, uint32_t PointCount, uint32_t Color);
int32_t  BSP_LCD_DrawEllipse(uint32_t Instance, int Xpos, int Ypos, int XRadius, int YRadius, uint32_t Color);
int32_t  BSP_LCD_FillRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height, uint32_t Color);
int32_t  BSP_LCD_FillRGBRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t *pData, uint32_t Width, uint32_t Height);
int32_t  BSP_LCD_FillCircle(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Radius, uint32_t Color);
int32_t  BSP_LCD_FillPolygon(uint32_t Instance, pPoint Points, uint32_t PointCount, uint32_t Color);
int32_t  BSP_LCD_FillEllipse(uint32_t Instance, int Xpos, int Ypos, int XRadius, int YRadius, uint32_t Color);
int32_t	 BSP_LCD_FillTriangle(uint32_t Instance, pPoint Points, uint32_t Color);
int32_t  BSP_LCD_DrawBitmap(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t *pBmp);


#if (USE_HAL_SRAM_REGISTER_CALLBACKS == 1)
int32_t  BSP_LCD_RegisterDefaultMspCallbacks(uint32_t Instance);
int32_t  BSP_LCD_RegisterMspCallbacks(uint32_t Instance, BSP_LCD_Cb_t *Callback);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
HAL_StatusTypeDef MX_FMC_BANK1_Init(SRAM_HandleTypeDef *hSram);



#ifdef __cplusplus
}
#endif

#endif /* STM32L562E_DISCOVERY_LCD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
