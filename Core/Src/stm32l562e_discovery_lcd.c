/**
  ******************************************************************************
  * @file    stm32l562e_discovery_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on STM32L562E-DISCOVERY board.
  @verbatim
  1. How To use this driver:
  --------------------------
     - This driver is used to drive indirectly an LCD TFT.
     - This driver supports the ST7789H2 LCD mounted on board.
     - The ST7789H2 component driver MUST be included with this driver.

  2. Driver description:
  ----------------------
    + Initialization steps:
       o Initialize the LCD using the BSP_LCD_Init() function. You can select
         display orientation with "Orientation" parameter (portrait, landscape,
         portrait with 180 degrees rotation or landscape with 180 degrees
         rotation).
       o Call BSP_LCD_GetXSize() and BSP_LCD_GetYsize() to get respectively
         width and height in pixels of LCD in the current orientation.
       o Call BSP_LCD_SetBrightness() and BSP_LCD_GetBrightness() to
         respectively set and get LCD brightness.
       o Call BSP_LCD_SetActiveLayer() to select the current active layer.
       o Call BSP_LCD_GetFormat() to get LCD pixel format supported.
       o Call BSP_LCD_SetFuncDriver() to initialize BSP driver.
       o Call BSP_LCD_Clear() to fill the entire screen with a single color.
       o Call BSP_LCD_DisplayOn() and BSP_LCD_DisplayOff() to respectively
         switch on and switch off the LCD display.

    + Write text on LCD:
       o Call BSP_LCD_SetTextColor() and BSP_LCD_GetTextColor() to respectively set
         and get the color of the text.
       o Call BSP_LCD_SetBackColor() and BSP_LCD_GetBackColor() to respectively set
         and get the background color of the text.
       o Call BSP_LCD_SetFont() and BSP_LCD_GetFont() to respectively set
         and get the text font.
       o Call BSP_LCD_ClearStringLine() to clear a text line by filling it
         with the background color.
       o Call BSP_LCD_DisplayStringAtLine() to write text at a given line.
       o Call BSP_LCD_DisplayStringAt() to write text at given X and Y coordinates.
       o Call BSP_LCD_DisplayChar() to write a single character.

    + Display shapes on LCD:
       o Call BSP_LCD_WritePixel() and BSP_LCD_ReadPixel() to respectively write
         and read a pixel.
       o Call BSP_LCD_DrawLine() to draw a line.
       o Call BSP_LCD_DrawHLine() to draw a horizontal line.
       o Call BSP_LCD_DrawVLine() to draw a vertical line.
       o Call BSP_LCD_DrawRect() to draw a rectangle.
       o Call BSP_LCD_FillRect() to draw a filled rectangle.
       o Call BSP_LCD_FillRGBRect() to draw a filled rectangle with RGB buffer.
       o Call BSP_LCD_DrawCircle() to draw a circle.
       o Call BSP_LCD_FillCircle() to draw a filled circle.
       o Call BSP_LCD_DrawEllipse() to draw an ellipse.
       o Call BSP_LCD_FillEllipse() to draw a filled ellipse.
       o Call BSP_LCD_DrawPolygon() to draw a polygon.
       o Call BSP_LCD_FillPolygon() to draw a filled polygon.
			 o Call BSP_LCD_FillTriangle() to draw a filled triangle.
       o Call BSP_LCD_DrawBitmap() to draw a bitmap.

    + De-initialization steps:
       o De-initialize the LCD using the BSP_LCD_DeInit() function.

  @endverbatim
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l562e_discovery_lcd.h"
#include "../Fonts/font24.h"
#include "../Fonts/font20.h"
#include "../Fonts/font16.h"
#include "../Fonts/font12.h"
#include "../Fonts/font8.h"

/** @defgroup STM32L562E-DK_LCD_Private_Defines STM32L562E-DK LCD Private Defines
  * @{
  */
#define LCD_POWER_GPIO_PORT               GPIOH
#define LCD_POWER_GPIO_PIN                GPIO_PIN_0
#define LCD_POWER_GPIO_CLOCK_ENABLE()     __HAL_RCC_GPIOH_CLK_ENABLE()
#define LCD_RESET_GPIO_PORT               GPIOF
#define LCD_RESET_GPIO_PIN                GPIO_PIN_14
#define LCD_RESET_GPIO_CLOCK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define LCD_BACKLIGHT_GPIO_PORT           GPIOE
#define LCD_BACKLIGHT_GPIO_PIN            GPIO_PIN_1
#define LCD_BACKLIGHT_GPIO_CLOCK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()

#define LCD_REGISTER_ADDR FMC_BANK1_1
#define LCD_DATA_ADDR     (FMC_BANK1_1 | 0x00000002UL)

#define LCD_FMC_ADDRESS   				1U
#define BSP_LCD_MAX_LAYERS_NBR    2U

#if (USE_HAL_SRAM_REGISTER_CALLBACKS == 1)
static uint32_t Lcd_IsSramMspCbValid[LCD_INSTANCES_NBR] = {0};
#endif


/** @addtogroup STM32L562E-DK_LCD_Exported_Variables
  * @{
  */
SRAM_HandleTypeDef	hlcd_sram[LCD_INSTANCES_NBR] = {0};
void								*Lcd_CompObj[LCD_INSTANCES_NBR] = {NULL};
LCD_Drv_t						*Lcd_Drv[LCD_INSTANCES_NBR] = {NULL};

/**
  * @brief  Current Drawing Layer properties variable
  */
static BSP_LCD_Ctx_t DrawProp[BSP_LCD_MAX_LAYERS_NBR];




/** @defgroup STM32L562E-DK_LCD_Private_FunctionPrototypes STM32L562E-DK LCD Private Function Prototypes
  * @{
  */
static int32_t ST7789H2_Probe(uint32_t Orientation);
static void    ST7789H2_PowerUp(void);
static void    ST7789H2_PowerDown(void);

static int32_t LCD_FMC_Init(void);
static int32_t LCD_FMC_DeInit(void);
static int32_t LCD_FMC_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint32_t Length);
static int32_t LCD_FMC_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint32_t Length);
static int32_t LCD_FMC_Send(uint8_t *pData, uint32_t Length);
static int32_t LCD_FMC_GetTick(void);
static void    FMC_MspInit(SRAM_HandleTypeDef *hSram);
static void    FMC_MspDeInit(SRAM_HandleTypeDef *hSram);
/**
  * @}
  */

/** @addtogroup STM32L562E-DK_LCD_Exported_Functions
  * @{
  */
/**
  * @brief  Initialize the LCD.
  * @param  Instance LCD Instance.
  * @param  Orientation LCD_ORIENTATION_PORTRAIT, LCD_ORIENTATION_LANDSCAPE,
  *                     LCD_ORIENTATION_PORTRAIT_ROT180 or LCD_ORIENTATION_LANDSCAPE_ROT180.
  * @retval BSP status.
  */
int32_t BSP_LCD_Init(uint32_t Instance, uint32_t Orientation)
{
  int32_t status = BSP_ERROR_NONE;

  if ((Instance >= LCD_INSTANCES_NBR) || (Orientation > LCD_ORIENTATION_LANDSCAPE_ROT180)) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Power up LCD */
    ST7789H2_PowerUp();

    /* Probe the LCD driver */
    if (ST7789H2_Probe(Orientation) != BSP_ERROR_NONE) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  De-Initialize the LCD.
  * @param  Instance LCD Instance.
  * @retval BSP status.
  */
int32_t BSP_LCD_DeInit(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* De-Init the LCD driver */
    if (Lcd_Drv[Instance]->DeInit(Lcd_CompObj[Instance]) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
    /* Power down LCD */
    ST7789H2_PowerDown();

    /* DeInit LCD pins */
    LCD_FMC_DeInit();
  }

  return status;
}

/**
  * @brief  Set the display on.
  * @param  Instance LCD Instance.
  * @retval BSP status.
  */
int32_t BSP_LCD_DisplayOn(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Set the display on */
    if (Lcd_Drv[Instance]->DisplayOn(Lcd_CompObj[Instance]) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Set the display off.
  * @param  Instance LCD Instance.
  * @retval BSP status.
  */
int32_t BSP_LCD_DisplayOff(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Set the display off */
    if (Lcd_Drv[Instance]->DisplayOff(Lcd_CompObj[Instance]) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Set the display brightness.
  * @param  Instance LCD Instance.
  * @param  Brightness [00: Min (black), 100 Max].
  * @retval BSP status.
  */
int32_t BSP_LCD_SetBrightness(uint32_t Instance, uint32_t Brightness)
{
  int32_t status = BSP_ERROR_NONE;

  if ((Instance >= LCD_INSTANCES_NBR) || (Brightness > 100U)) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Set the display brightness */
    if (Lcd_Drv[Instance]->SetBrightness(Lcd_CompObj[Instance], Brightness) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Get the display brightness.
  * @param  Instance LCD Instance.
  * @param  Brightness [00: Min (black), 100 Max].
  * @retval BSP status.
  */
int32_t BSP_LCD_GetBrightness(uint32_t Instance, uint32_t *Brightness)
{
  int32_t status = BSP_ERROR_NONE;

  if ((Instance >= LCD_INSTANCES_NBR) || (Brightness == NULL)) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Get the display brightness */
    if (Lcd_Drv[Instance]->GetBrightness(Lcd_CompObj[Instance], Brightness) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Get the LCD X size.
  * @param  Instance LCD Instance.
  * @param  Xsize LCD X size.
  * @retval BSP status.
  */
int32_t BSP_LCD_GetXSize(uint32_t Instance, uint32_t *Xsize)
{
  int32_t status = BSP_ERROR_NONE;

  if ((Instance >= LCD_INSTANCES_NBR) || (Xsize == NULL)) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Get the display Xsize */
    if (Lcd_Drv[Instance]->GetXSize(Lcd_CompObj[Instance], Xsize) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Get the LCD Y size.
  * @param  Instance LCD Instance.
  * @param  Ysize LCD Y size.
  * @retval BSP status.
  */
int32_t BSP_LCD_GetYSize(uint32_t Instance, uint32_t *Ysize)
{
  int32_t status = BSP_ERROR_NONE;

  if ((Instance >= LCD_INSTANCES_NBR) || (Ysize == NULL)) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Get the display Ysize */
    if (Lcd_Drv[Instance]->GetYSize(Lcd_CompObj[Instance], Ysize) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Get pixel format supported by LCD.
  * @param  Instance LCD Instance.
  * @param  Format Pointer on pixel format.
  * @retval BSP status.
  */
int32_t BSP_LCD_GetFormat(uint32_t Instance, uint32_t *Format)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Get pixel format supported by LCD */
    *Format = LCD_PIXEL_FORMAT_RGB565;
  }

  return status;  
}

/**
  * @brief  Link board LCD drivers to STM32 LCD Utility drivers
  * @param  pDrv Structure of LCD functions
  * @retval BSP status.
  */
int32_t BSP_LCD_SetFuncDriver(void)
{
  int32_t status = BSP_ERROR_NONE;

  DrawProp->LcdLayer = 0;
  DrawProp->LcdDevice = 0;
  status = BSP_LCD_GetXSize(0, &DrawProp->LcdXsize);
  if(status == BSP_ERROR_NONE) {
	  status = BSP_LCD_GetYSize(0, &DrawProp->LcdYsize);
  }
  if(status == BSP_ERROR_NONE) {
	  status = BSP_LCD_GetFormat(0, &DrawProp->LcdPixelFormat);
  }

  return status;
}

/**
  * @brief  Sets the LCD text color.
  * @param  Color  Text color code
  */
void BSP_LCD_SetTextColor(uint32_t Color)
{
  DrawProp[DrawProp->LcdLayer].TextColor = Color;
}

/**
  * @brief  Gets the LCD text color.
  * @retval Used text color.
  */
uint32_t BSP_LCD_GetTextColor(void)
{
  return DrawProp[DrawProp->LcdLayer].TextColor;
}

/**
  * @brief  Sets the LCD background color.
  * @param  Color  Layer background color code
  */
void BSP_LCD_SetBackColor(uint32_t Color)
{
  DrawProp[DrawProp->LcdLayer].BackColor = Color;
}

/**
  * @brief  Gets the LCD background color.
  * @retval Used background color
  */
uint32_t BSP_LCD_GetBackColor(void)
{
  return DrawProp[DrawProp->LcdLayer].BackColor;
}

/**
  * @brief  Sets the LCD text font.
  * @param  fonts  Layer font to be used
  */
void BSP_LCD_SetFont(sFONT *fonts)
{
  DrawProp[DrawProp->LcdLayer].pFont = fonts;
}

/**
  * @brief  Gets the LCD text font.
  * @retval Used layer font
  */
sFONT *BSP_LCD_GetFont(void)
{
  return DrawProp[DrawProp->LcdLayer].pFont;
}

/**
  * @brief  Clears the whole currently active layer of LTDC.
  * @param  Color  Color of the background
  * @retval BSP status.
  */
int32_t BSP_LCD_Clear(uint32_t Instance, uint32_t Color)
{
  int32_t status = BSP_ERROR_NONE;

  /* Clear the LCD */
  status = BSP_LCD_FillRect(Instance, 0, 0, DrawProp->LcdXsize, DrawProp->LcdYsize, Color);

  return status;
}

/**
  * @brief  Clears the selected line in currently active layer.
  * @param  Line  Line to be cleared
  * @retval BSP status.
  */
int32_t BSP_LCD_ClearStringLine(uint32_t Instance, uint32_t Line)
{
  int32_t status = BSP_ERROR_NONE;

  /* Draw rectangle with background color */
  status = BSP_LCD_FillRect(Instance, 0, (Line * DrawProp[DrawProp->LcdLayer].pFont->Height), DrawProp->LcdXsize, DrawProp[DrawProp->LcdLayer].pFont->Height, DrawProp[DrawProp->LcdLayer].BackColor);

  return status;
}

/**
  * @brief  Displays a maximum of 60 characters on the LCD.
  * @param  Line: Line where to display the character shape
  * @param  ptr: Pointer to string to display on LCD
  * @retval BSP status.
  */
int32_t BSP_LCD_DisplayStringAtLine(uint32_t Instance, uint32_t Line, uint8_t *ptr, Text_AlignModeTypdef Mode)
{
  int32_t status = BSP_ERROR_NONE;

  status = BSP_LCD_DisplayStringAt(Instance, 0, LINE(Line), ptr, Mode);

  return status;
}

/**
  * @brief  Displays characters in currently active layer.
  * @param  Xpos X position (in pixel)
  * @param  Ypos Y position (in pixel)
  * @param  Text Pointer to string to display on LCD
  * @param  Mode Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  LEFT_MODE
  * @retval BSP status.
  */
int32_t BSP_LCD_DisplayStringAt(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode)
{
  uint32_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t  *ptr = Text;
  int32_t status = BSP_ERROR_NONE;

  /* Get the text size */
  while (*ptr++) size ++ ;

  /* Characters number per line */
  xsize = (DrawProp->LcdXsize/DrawProp[DrawProp->LcdLayer].pFont->Width);

  switch (Mode) {
		case CENTER_MODE:
		{
			refcolumn = Xpos + ((xsize - size)* DrawProp[DrawProp->LcdLayer].pFont->Width) / 2;
			break;
		}
		case LEFT_MODE:
		{
			refcolumn = Xpos;
			break;
		}
		case RIGHT_MODE:
		{
			refcolumn = - Xpos + ((xsize - size)*DrawProp[DrawProp->LcdLayer].pFont->Width);
			break;
		}
		default:
		{
			refcolumn = Xpos;
			break;
		}
  }

  /* Check that the Start column is located in the screen */
  if ((refcolumn < 1) || (refcolumn >= 0x8000)) {
    refcolumn = 1;
  }

  /* Send the string character by character on LCD */
  while ((*Text != 0) & (((DrawProp->LcdXsize - (i*DrawProp[DrawProp->LcdLayer].pFont->Width)) & 0xFFFF) >= DrawProp[DrawProp->LcdLayer].pFont->Width)) {
    /* Display one character on LCD */
		if(status == BSP_ERROR_NONE) {
			status = BSP_LCD_DisplayChar(Instance, refcolumn, Ypos, *Text);
		}
    /* Decrement the column position by 16 */
    refcolumn += DrawProp[DrawProp->LcdLayer].pFont->Width;

    /* Point on the next character */
    Text++;
    i++;
  }

  return status;
}

/**
  * @brief  Displays one character in currently active layer.
  * @param  Xpos Start column address
  * @param  Ypos Line where to display the character shape.
  * @param  Ascii Character ascii code
  *           This parameter must be a number between Min_Data = 0x20 and Max_Data = 0x7E
  * @retval BSP status.
  */
int32_t BSP_LCD_DisplayChar(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t Ascii)
{
  int32_t status = BSP_ERROR_NONE;

  status = BSP_LCD_DrawChar(Instance, Xpos, Ypos, &DrawProp[DrawProp->LcdLayer].pFont->table[(Ascii-' ') *\
  DrawProp[DrawProp->LcdLayer].pFont->Height * ((DrawProp[DrawProp->LcdLayer].pFont->Width + 7) / 8)]);

  return status;
}

/**
  * @brief  Draws a character on LCD.
  * @param  Xpos  Line where to display the character shape
  * @param  Ypos  Start column address
  * @param  pData Pointer to the character data
  * @retval BSP status.
  */
int32_t BSP_LCD_DrawChar(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, const uint8_t *pData)
{
  uint32_t i = 0, j = 0, offset;
  uint32_t height, width;
  uint8_t  *pchar;
  uint32_t line;
  int32_t  status = BSP_ERROR_NONE;

  height = DrawProp[DrawProp->LcdLayer].pFont->Height;
  width  = DrawProp[DrawProp->LcdLayer].pFont->Width;
  uint16_t rgb565[24];
  uint32_t argb8888[24];

  offset =  8 *((width + 7)/8) -  width ;

  for(i = 0; i < height; i++) {
    pchar = ((uint8_t *)pData + (width + 7)/8 * i);

    switch(((width + 7)/8)) {
			case 1:
				line = pchar[0];
				break;
			case 2:
				line = (pchar[0]<< 8) | pchar[1];
				break;
			case 3:
			default:
				line = (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];
				break;
    }

    if(DrawProp[DrawProp->LcdLayer].LcdPixelFormat == LCD_PIXEL_FORMAT_RGB565) {
      for (j = 0; j < width; j++) {
        if(line & (1 << (width- j + offset- 1))) {
          rgb565[j] = CONVERTARGB88882RGB565(DrawProp[DrawProp->LcdLayer].TextColor);
        } else {
          rgb565[j] = CONVERTARGB88882RGB565(DrawProp[DrawProp->LcdLayer].BackColor);
        }
      }
      if(status == BSP_ERROR_NONE) {
    	  status = BSP_LCD_FillRGBRect(Instance, Xpos,  Ypos++, (uint8_t*)&rgb565[0], width, 1);
      }
    } else {
      for (j = 0; j < width; j++) {
        if(line & (1 << (width- j + offset- 1))) {
          argb8888[j] = DrawProp[DrawProp->LcdLayer].TextColor;
        } else {
          argb8888[j] = DrawProp[DrawProp->LcdLayer].BackColor;
        }
      }
      if(status == BSP_ERROR_NONE) {
    	  status = BSP_LCD_FillRGBRect(Instance, Xpos,  Ypos++, (uint8_t*)&argb8888[0], width, 1);
      }
    }
  }

  return status;
}

/**
  * @brief  Read a pixel on LCD.
  * @param  Instance LCD Instance.
  * @param  Xpos X position.
  * @param  Ypos Y position.
  * @param  Color Pointer to the pixel.
  * @retval BSP status.
  */
int32_t BSP_LCD_ReadPixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t *Color)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Read pixel on LCD */
    if (Lcd_Drv[Instance]->GetPixel(Lcd_CompObj[Instance], Xpos, Ypos, Color) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Write a pixel on LCD.
  * @param  Instance LCD Instance.
  * @param  Xpos X position.
  * @param  Ypos Y position.
  * @param  Color Pixel.
  * @retval BSP status.
  */
int32_t BSP_LCD_WritePixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Color)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Write pixel on LCD */
    if (Lcd_Drv[Instance]->SetPixel(Lcd_CompObj[Instance], Xpos, Ypos, CONVERTARGB88882RGB565(Color)) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Draws an uni-line (between two points) in currently active layer.
  * @param  Xpos1 Point 1 X position
  * @param  Ypos1 Point 1 Y position
  * @param  Xpos2 Point 2 X position
  * @param  Ypos2 Point 2 Y position
  * @param  Color Draw color
  * @retval BSP status.
  */
int32_t BSP_LCD_DrawLine(uint32_t Instance, uint32_t Xpos1, uint32_t Ypos1, uint32_t Xpos2, uint32_t Ypos2, uint32_t Color)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;
  int32_t x_diff, y_diff;
  int32_t status = BSP_ERROR_NONE;

  x_diff = Xpos2 - Xpos1;
  y_diff = Ypos2 - Ypos1;

  deltax = ABS(x_diff);					/* The absolute difference between the x's */
  deltay = ABS(y_diff);					/* The absolute difference between the y's */
  x = Xpos1;										/* Start x off at the first pixel */
  y = Ypos1;										/* Start y off at the first pixel */

  if (Xpos2 >= Xpos1) {					/* The x-values are increasing */
    xinc1 = 1;
    xinc2 = 1;
  } else {											/* The x-values are decreasing */
    xinc1 = -1;
    xinc2 = -1;
  }

  if (Ypos2 >= Ypos1) {					/* The y-values are increasing */
    yinc1 = 1;
    yinc2 = 1;
  } else {											/* The y-values are decreasing */
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay) {				/* There is at least one x-value for every y-value */
    xinc1 = 0;									/* Don't change the x when numerator >= denominator */
    yinc2 = 0;									/* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;					/* There are more x-values than y-values */
  } else {											/* There is at least one y-value for every x-value */
    xinc2 = 0;									/* Don't change the x for every iteration */
    yinc1 = 0;									/* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;					/* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		if(status == BSP_ERROR_NONE) {
			status = BSP_LCD_WritePixel(Instance, x, y, Color);   /* Draw the current pixel */
		}
    num += numadd;							/* Increase the numerator by the top of the fraction */
    if (num >= den) {						/* Check if numerator >= denominator */
      num -= den;								/* Calculate the new numerator value */
      x += xinc1;								/* Change the x as appropriate */
      y += yinc1;								/* Change the y as appropriate */
    }
    x += xinc2;									/* Change the x as appropriate */
    y += yinc2;									/* Change the y as appropriate */
  }

  return status;
}

/**
  * @brief  Draw a horizontal line on LCD.
  * @param  Instance LCD Instance.
  * @param  Xpos X position.
  * @param  Ypos Y position.
  * @param  Length Length of the line.
  * @param  Color Color of the line.
  * @retval BSP status.
  */
int32_t BSP_LCD_DrawHLine(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Length, uint32_t Color)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Draw the horizontal line on LCD */
    if (Lcd_Drv[Instance]->DrawHLine(Lcd_CompObj[Instance], Xpos, Ypos, Length, CONVERTARGB88882RGB565(Color)) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Draw a vertical line on LCD.
  * @param  Instance LCD Instance.
  * @param  Xpos X position.
  * @param  Ypos Y position.
  * @param  Length Length of the line.
  * @param  Color Color of the line.
  * @retval BSP status.
  */
int32_t BSP_LCD_DrawVLine(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Length, uint32_t Color)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Draw the vertical line on LCD */
    if (Lcd_Drv[Instance]->DrawVLine(Lcd_CompObj[Instance], Xpos, Ypos, Length, CONVERTARGB88882RGB565(Color)) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Draws a rectangle in currently active layer.
  * @param  Xpos X position
  * @param  Ypos Y position
  * @param  Width  Rectangle width
  * @param  Height Rectangle height
  * @param  Color  Draw color
  * @retval BSP status.
  */
int32_t BSP_LCD_DrawRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height, uint32_t Color)
{
  int32_t status = BSP_ERROR_NONE;

  /* Draw horizontal lines */
  status = BSP_LCD_DrawHLine(Instance, Xpos, Ypos, Width, Color);
  if(status == BSP_ERROR_NONE) {
	  status = BSP_LCD_DrawHLine(Instance, Xpos, (Ypos+ Height - 1U), Width, Color);
  }

  /* Draw vertical lines */
  if(status == BSP_ERROR_NONE) {
	  status = BSP_LCD_DrawVLine(Instance, Xpos, Ypos, Height, Color);
  }
  if(status == BSP_ERROR_NONE) {
	  status = BSP_LCD_DrawVLine(Instance, (Xpos + Width - 1U), Ypos, Height, Color);
  }

  return status;
}

/**
  * @brief  Draws a circle in currently active layer.
  * @param  Xpos    X position
  * @param  Ypos    Y position
  * @param  Radius  Circle radius
  * @param  Color   Draw color
  * @retval BSP status.
  */
int32_t BSP_LCD_DrawCircle(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Radius, uint32_t Color)
{
  int32_t   decision;  /* Decision Variable */
  uint32_t  current_x; /* Current X Value */
  uint32_t  current_y; /* Current Y Value */
  int32_t   status = BSP_ERROR_NONE;

  decision = 3 - (Radius << 1);
  current_x = 0;
  current_y = Radius;

  while (current_x <= current_y) {
    if((Ypos - current_y) < DrawProp->LcdYsize) {
      if((Xpos + current_x) < DrawProp->LcdXsize) {
      	status = BSP_LCD_WritePixel(Instance, (Xpos + current_x), (Ypos - current_y), Color);
      }
      if((Xpos - current_x) < DrawProp->LcdXsize) {
    	  if(status == BSP_ERROR_NONE) {
    		  status = BSP_LCD_WritePixel(Instance, (Xpos - current_x), (Ypos - current_y), Color);
    	  }
      }
    }

    if((Ypos - current_x) < DrawProp->LcdYsize) {
      if((Xpos + current_y) < DrawProp->LcdXsize) {
				if(status == BSP_ERROR_NONE) {
					status = BSP_LCD_WritePixel(Instance, (Xpos + current_y), (Ypos - current_x), Color);
				}
      }
      if((Xpos - current_y) < DrawProp->LcdXsize) {
				if(status == BSP_ERROR_NONE) {
					status = BSP_LCD_WritePixel(Instance, (Xpos - current_y), (Ypos - current_x), Color);
				}
      }
    }

    if((Ypos + current_y) < DrawProp->LcdYsize) {
      if((Xpos + current_x) < DrawProp->LcdXsize) {
				if(status == BSP_ERROR_NONE) {
					status = BSP_LCD_WritePixel(Instance, (Xpos + current_x), (Ypos + current_y), Color);
				}
      }
      if((Xpos - current_x) < DrawProp->LcdXsize) {
				if(status == BSP_ERROR_NONE) {
					status = BSP_LCD_WritePixel(Instance, (Xpos - current_x), (Ypos + current_y), Color);
				}
      }
    }

    if((Ypos + current_x) < DrawProp->LcdYsize) {
      if((Xpos + current_y) < DrawProp->LcdXsize) {
				if(status == BSP_ERROR_NONE) {
					status = BSP_LCD_WritePixel(Instance, (Xpos + current_y), (Ypos + current_x), Color);
				}
      }
      if((Xpos - current_y) < DrawProp->LcdXsize) {
				if(status == BSP_ERROR_NONE) {
					status = BSP_LCD_WritePixel(Instance, (Xpos - current_y), (Ypos + current_x), Color);
				}
      }
    }

    if (decision < 0) {
      decision += (current_x << 2) + 6;
    } else {
      decision += ((current_x - current_y) << 2) + 10;
      current_y--;
    }
    current_x++;
  }

  return status;
}

/**
  * @brief  Draws an poly-line (between many points) in currently active layer.
  * @param  Points      Pointer to the points array
  * @param  PointCount  Number of points
  * @param  Color       Draw color
  * @retval BSP status.
  */
int32_t BSP_LCD_DrawPolygon(uint32_t Instance, pPoint Points, uint32_t PointCount, uint32_t Color)
{
  int16_t x_pos = 0, y_pos = 0;
  int32_t status = BSP_ERROR_NONE;

  if(PointCount < 2) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
	  if(status == BSP_ERROR_NONE) {
		  status = BSP_LCD_DrawLine(Instance, Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y, Color);
	  }
	  while(--PointCount) {
	    x_pos = Points->X;
	    y_pos = Points->Y;
	    Points++;
			if(status == BSP_ERROR_NONE) {
				status = BSP_LCD_DrawLine(Instance, x_pos, y_pos, Points->X, Points->Y, Color);
			}
	  }
  }

  return status;
}

/**
  * @brief  Draws an ellipse on LCD in currently active layer.
  * @param  Xpos    X position
  * @param  Ypos    Y position
  * @param  XRadius Ellipse X radius
  * @param  YRadius Ellipse Y radius
  * @param  Color   Draw color
  * @retval BSP status.
  */
int32_t BSP_LCD_DrawEllipse(uint32_t Instance, int Xpos, int Ypos, int XRadius, int YRadius, uint32_t Color)
{
  int x_pos = 0, y_pos = -YRadius, err = 2-2*XRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;
  int32_t status = BSP_ERROR_NONE;

  rad1 = XRadius;
  rad2 = YRadius;

  k = (float)(rad2/rad1);

  do {
	  if(status == BSP_ERROR_NONE) {
		  status = BSP_LCD_WritePixel(Instance, (Xpos-(uint32_t)(x_pos/k)), (Ypos + y_pos), Color);
	  }
	  if(status == BSP_ERROR_NONE) {
		  status = BSP_LCD_WritePixel(Instance, (Xpos+(uint32_t)(x_pos/k)), (Ypos + y_pos), Color);
	  }
	  if(status == BSP_ERROR_NONE) {
		  status = BSP_LCD_WritePixel(Instance, (Xpos+(uint32_t)(x_pos/k)), (Ypos - y_pos), Color);
	  }
	  if(status == BSP_ERROR_NONE) {
		  status = BSP_LCD_WritePixel(Instance, (Xpos-(uint32_t)(x_pos/k)), (Ypos - y_pos), Color);
	  }

    e2 = err;
    if (e2 <= x_pos) {
      err += ++x_pos*2+1;
      if (-y_pos == x_pos && e2 <= y_pos) e2 = 0;
    }
    if (e2 > y_pos) {
      err += ++y_pos*2+1;
    }
  } while (y_pos <= 0);

  return status;
}

/**
  * @brief  Fill rectangle with RGB buffer.
  * @param  Instance LCD Instance.
  * @param  Xpos X position on LCD.
  * @param  Ypos Y position on LCD.
  * @param  pData Pointer on RGB pixels buffer.
  * @param  Width Width of the rectangle.
  * @param  Height Height of the rectangle.
  * @retval BSP status.
  */
int32_t BSP_LCD_FillRGBRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t *pData, uint32_t Width, uint32_t Height)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Fill rectangle on LCD */
    if (Lcd_Drv[Instance]->FillRGBRect(Lcd_CompObj[Instance], Xpos, Ypos, pData, Width, Height) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Draw and fill a rectangle on LCD.
  * @param  Instance LCD Instance.
  * @param  Xpos X position.
  * @param  Ypos Y position.
  * @param  Width Width of the rectangle.
  * @param  Height Height of the rectangle.
  * @param  Color Color of the rectangle.
  * @retval BSP status.
  */
int32_t BSP_LCD_FillRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height, uint32_t Color)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Draw an fill rectangle on LCD */
    if (Lcd_Drv[Instance]->FillRect(Lcd_CompObj[Instance], Xpos, Ypos, Width, Height, CONVERTARGB88882RGB565(Color)) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Draws a full circle in currently active layer.
  * @param  Xpos   X position
  * @param  Ypos   Y position
  * @param  Radius Circle radius
  * @param  Color  Draw color
  * @retval BSP status.
  */
int32_t BSP_LCD_FillCircle(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Radius, uint32_t Color)
{
  int32_t   decision;  /* Decision Variable */
  uint32_t  current_x; /* Current X Value */
  uint32_t  current_y; /* Current Y Value */
  int32_t 	status = BSP_ERROR_NONE;

  decision = 3 - (Radius << 1);

  current_x = 0;
  current_y = Radius;

  while (current_x <= current_y)
  {
    if(current_y > 0) {
      if(current_y >= Xpos) {
    	  if(status == BSP_ERROR_NONE) {
    		  status = BSP_LCD_DrawHLine(Instance, 0, Ypos + current_x, 2*current_y - (current_y - Xpos), Color);
    	  }
    	  if(status == BSP_ERROR_NONE) {
    		  status = BSP_LCD_DrawHLine(Instance, 0, Ypos - current_x, 2*current_y - (current_y - Xpos), Color);
    	  }
      } else {
    	  if(status == BSP_ERROR_NONE) {
    		  status = BSP_LCD_DrawHLine(Instance, Xpos - current_y, Ypos + current_x, 2*current_y, Color);
    	  }
    	  if(status == BSP_ERROR_NONE) {
    		  status = BSP_LCD_DrawHLine(Instance, Xpos - current_y, Ypos - current_x, 2*current_y, Color);
    	  }
      }
    }

    if(current_x > 0) {
      if(current_x >= Xpos) {
    	  if(status == BSP_ERROR_NONE) {
    		  status = BSP_LCD_DrawHLine(Instance, 0, Ypos - current_y, 2*current_x - (current_x - Xpos), Color);
    	  }
    	  if(status == BSP_ERROR_NONE) {
    		  status = BSP_LCD_DrawHLine(Instance, 0, Ypos + current_y, 2*current_x - (current_x - Xpos), Color);
    	  }
      } else {
    	  if(status == BSP_ERROR_NONE) {
    		  status = BSP_LCD_DrawHLine(Instance, Xpos - current_x, Ypos - current_y, 2*current_x, Color);
    	  }
    	  if(status == BSP_ERROR_NONE) {
    		  status = BSP_LCD_DrawHLine(Instance, Xpos - current_x, Ypos + current_y, 2*current_x, Color);
    	  }
      }
    }
    if (decision < 0) {
      decision += (current_x << 2) + 6;
    } else {
      decision += ((current_x - current_y) << 2) + 10;
      current_y--;
    }
    current_x++;
  }

  if(status == BSP_ERROR_NONE) {
	  status = BSP_LCD_DrawCircle(Instance, Xpos, Ypos, Radius, Color);
  }

  return status;
}

/**
  * @brief  Draws a full poly-line (between many points) in currently active layer.
  * @param  Points     Pointer to the points array
  * @param  PointCount Number of points
  * @param  Color      Draw color
  * @retval BSP status.
  */
int32_t BSP_LCD_FillPolygon(uint32_t Instance, pPoint Points, uint32_t PointCount, uint32_t Color)
{
  int16_t X = 0, Y = 0, X2 = 0, Y2 = 0, x_center = 0, y_center = 0, x_first = 0, y_first = 0, pixel_x = 0, pixel_y = 0, counter = 0;
  uint32_t  image_left = 0, image_right = 0, image_top = 0, image_bottom = 0;
  Point Points_cur[3];
  int32_t status = BSP_ERROR_NONE;

  image_left = image_right = Points->X;
  image_top= image_bottom = Points->Y;

  for(counter = 1; counter < PointCount; counter++) {
    pixel_x = POLY_X(counter);
    if(pixel_x < image_left) {
      image_left = pixel_x;
    }
    if(pixel_x > image_right) {
      image_right = pixel_x;
    }

    pixel_y = POLY_Y(counter);
    if(pixel_y < image_top) {
      image_top = pixel_y;
    }
    if(pixel_y > image_bottom) {
      image_bottom = pixel_y;
    }
  }

  if(PointCount < 2) {
    status = BSP_ERROR_WRONG_PARAM;
  }

  x_center = (image_left + image_right)/2;
  y_center = (image_bottom + image_top)/2;

  x_first = Points->X;
  y_first = Points->Y;

  while(--PointCount) {
    X = Points->X;
    Y = Points->Y;
    Points++;
    X2 = Points->X;
    Y2 = Points->Y;
    Points_cur[0].X = X;
    Points_cur[0].Y = Y;
    Points_cur[1].X = X2;
    Points_cur[1].Y = Y2;
    Points_cur[2].X = x_center;
    Points_cur[2].Y = y_center;
    if(status == BSP_ERROR_NONE) {
    	status = BSP_LCD_FillTriangle(Instance, (pPoint)&Points_cur, Color);
    }

    Points_cur[1].X = x_center;
    Points_cur[1].Y = y_center;
    Points_cur[2].X = X2;
    Points_cur[2].Y = Y2;
    if(status == BSP_ERROR_NONE) {
    	status = BSP_LCD_FillTriangle(Instance, (pPoint)&Points_cur, Color);
    }

    Points_cur[0].X = x_center;
    Points_cur[0].Y = y_center;
    Points_cur[1].X = X2;
    Points_cur[1].Y = Y2;
    Points_cur[2].X = X;
    Points_cur[2].Y = Y;
    if(status == BSP_ERROR_NONE) {
    	status = BSP_LCD_FillTriangle(Instance, (pPoint)&Points_cur, Color);
    }
  }

	Points_cur[0].X = x_first;
	Points_cur[0].Y = y_first;
	Points_cur[1].X = X2;
	Points_cur[1].Y = Y2;
	Points_cur[2].X = x_center;
	Points_cur[2].Y = y_center;
	if(status == BSP_ERROR_NONE) {
		status = BSP_LCD_FillTriangle(Instance, (pPoint)&Points_cur, Color);
	}

	Points_cur[1].X = x_center;
	Points_cur[1].Y = y_center;
	Points_cur[2].X = X2;
	Points_cur[2].Y = Y2;
	if(status == BSP_ERROR_NONE) {
		status = BSP_LCD_FillTriangle(Instance, (pPoint)&Points_cur, Color);
	}

	Points_cur[0].X = x_center;
	Points_cur[0].Y = y_center;
	Points_cur[1].X = X2;
	Points_cur[1].Y = Y2;
	Points_cur[2].X = x_first;
	Points_cur[2].Y = y_first;
	if(status == BSP_ERROR_NONE) {
		status = BSP_LCD_FillTriangle(Instance, (pPoint)&Points_cur, Color);
	}

	return status;
}

/**
  * @brief  Draws a full ellipse in currently active layer.
  * @param  Xpos    X position
  * @param  Ypos    Y position
  * @param  XRadius Ellipse X radius
  * @param  YRadius Ellipse Y radius
  * @param  Color   Draw color
  * @retval BSP status.
  */
int32_t BSP_LCD_FillEllipse(uint32_t Instance, int Xpos, int Ypos, int XRadius, int YRadius, uint32_t Color)
{
  int x_pos = 0, y_pos = -YRadius, err = 2-2*XRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;
  int32_t 	status = BSP_ERROR_NONE;

  rad1 = XRadius;
  rad2 = YRadius;

  k = (float)(rad2/rad1);

  do {
	  if(status == BSP_ERROR_NONE) {
		  status = BSP_LCD_DrawHLine(Instance, (Xpos-(uint32_t)(x_pos/k)), (Ypos + y_pos), (2*(uint32_t)(x_pos/k) + 1), Color);
	  }
	  if(status == BSP_ERROR_NONE) {
		  status = BSP_LCD_DrawHLine(Instance, (Xpos-(uint32_t)(x_pos/k)), (Ypos - y_pos), (2*(uint32_t)(x_pos/k) + 1), Color);
	  }

    e2 = err;
    if (e2 <= x_pos) {
      err += ++x_pos*2+1;
      if (-y_pos == x_pos && e2 <= y_pos) e2 = 0;
    }
    if (e2 > y_pos) err += ++y_pos*2+1;
  } while (y_pos <= 0);

  return status;
}

/**
  * @brief  Fills a triangle (between 3 points).
  * @param  Positions  pointer to riangle coordinates
  * @param  Color      Draw color
  * @retval BSP status.
  */
int32_t BSP_LCD_FillTriangle(uint32_t Instance, pPoint Points, uint32_t Color)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;
  int32_t x_diff, y_diff;
  int32_t status = BSP_ERROR_NONE;

  x_diff = Points[1].X - Points[0].X;
  y_diff = Points[1].Y - Points[0].Y;

  deltax = ABS(x_diff);									/* The absolute difference between the x's */
  deltay = ABS(y_diff);									/* The absolute difference between the y's */
  x = Points[0].X;											/* Start x off at the first pixel */
  y = Points[0].Y;											/* Start y off at the first pixel */

  if (Points[1].X >= Points[0].X) {			/* The x-values are increasing */
    xinc1 = 1;
    xinc2 = 1;
  } else {															/* The x-values are decreasing */
    xinc1 = -1;
    xinc2 = -1;
  }

  if (Points[1].Y >= Points[0].Y) {			/* The y-values are increasing */
    yinc1 = 1;
    yinc2 = 1;
  } else {															/* The y-values are decreasing */
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay) {								/* There is at least one x-value for every y-value */
    xinc1 = 0;													/* Don't change the x when numerator >= denominator */
    yinc2 = 0;													/* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;									/* There are more x-values than y-values */
  } else {															/* There is at least one y-value for every x-value */
    xinc2 = 0;													/* Don't change the x for every iteration */
    yinc1 = 0;													/* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;									/* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++) {
    status = BSP_LCD_DrawLine(Instance, x, y, Points[2].X, Points[2].Y, Color);

    num += numadd;											/* Increase the numerator by the top of the fraction */
    if (num >= den) {										/* Check if numerator >= denominator */
      num -= den;												/* Calculate the new numerator value */
      x += xinc1;												/* Change the x as appropriate */
      y += yinc1;												/* Change the y as appropriate */
    }
    x += xinc2;													/* Change the x as appropriate */
    y += yinc2;													/* Change the y as appropriate */
  }

  return status;
}

/**
  * @brief  Draw a bitmap on LCD.
  * @param  Instance LCD Instance.
  * @param  Xpos X position.
  * @param  Ypos Y position.
  * @param  pBmp Pointer to bitmap.
  * @retval BSP status.
  */
int32_t BSP_LCD_DrawBitmap(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t *pBmp)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    status = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Draw the bitmap on LCD */
    if (Lcd_Drv[Instance]->DrawBitmap(Lcd_CompObj[Instance], Xpos, Ypos, pBmp) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return status;
}





/**
  * @brief  MX FMC BANK1 initialization.
  * @param  hSram SRAM handle.
  * @retval HAL status.
  */
__weak HAL_StatusTypeDef MX_FMC_BANK1_Init(SRAM_HandleTypeDef *hSram)
{
  static FMC_NORSRAM_TimingTypeDef  SramTiming = {0};
  static FMC_NORSRAM_TimingTypeDef  ExtendedTiming = {0};

  /* SRAM device configuration */
  hSram->Init.DataAddressMux         = FMC_DATA_ADDRESS_MUX_DISABLE;
  hSram->Init.MemoryType             = FMC_MEMORY_TYPE_SRAM;
  hSram->Init.MemoryDataWidth        = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hSram->Init.BurstAccessMode        = FMC_BURST_ACCESS_MODE_DISABLE;
  hSram->Init.WaitSignalPolarity     = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hSram->Init.WaitSignalActive       = FMC_WAIT_TIMING_BEFORE_WS;
  hSram->Init.WriteOperation         = FMC_WRITE_OPERATION_ENABLE;
  hSram->Init.WaitSignal             = FMC_WAIT_SIGNAL_DISABLE;
  hSram->Init.ExtendedMode           = FMC_EXTENDED_MODE_DISABLE;
  hSram->Init.AsynchronousWait       = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hSram->Init.WriteBurst             = FMC_WRITE_BURST_DISABLE;
  hSram->Init.ContinuousClock        = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hSram->Init.WriteFifo              = FMC_WRITE_FIFO_DISABLE;
  hSram->Init.PageSize               = FMC_PAGE_SIZE_NONE;
  hSram->Init.NBLSetupTime           = FMC_NBL_SETUPTIME_0;
  hSram->Init.MaxChipSelectPulse     = DISABLE;
  hSram->Init.MaxChipSelectPulseTime = 1;

  SramTiming.AddressSetupTime      = 1;
  SramTiming.AddressHoldTime       = 1;
  SramTiming.DataSetupTime         = 32;
  SramTiming.DataHoldTime          = 0;
  SramTiming.BusTurnAroundDuration = 0;
  SramTiming.CLKDivision           = 2;
  SramTiming.DataLatency           = 2;
  SramTiming.AccessMode            = FMC_ACCESS_MODE_A;

  ExtendedTiming.AddressSetupTime      = 5;
  ExtendedTiming.AddressHoldTime       = 1;
  ExtendedTiming.DataSetupTime         = 3;
  ExtendedTiming.BusTurnAroundDuration = 2;
  ExtendedTiming.CLKDivision           = SramTiming.CLKDivision;
  ExtendedTiming.DataLatency           = SramTiming.DataLatency;
  ExtendedTiming.AccessMode            = SramTiming.AccessMode;

  return HAL_SRAM_Init(hSram, &SramTiming, &ExtendedTiming);
}

#if (USE_HAL_SRAM_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register Default LCD Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_LCD_RegisterDefaultMspCallbacks(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    ret = BSP_ERROR_WRONG_PARAM;
  } else {
    __HAL_SRAM_RESET_HANDLE_STATE(&hlcd_sram[Instance]);

    /* Register default MspInit/MspDeInit Callback */
    if (HAL_SRAM_RegisterCallback(&hlcd_sram[Instance], HAL_SRAM_MSP_INIT_CB_ID, FMC_MspInit) != HAL_OK) {
      ret = BSP_ERROR_PERIPH_FAILURE;
    } else if (HAL_SRAM_RegisterCallback(&hlcd_sram[Instance], HAL_SRAM_MSP_DEINIT_CB_ID, FMC_MspDeInit) != HAL_OK) {
      ret = BSP_ERROR_PERIPH_FAILURE;
    } else {
      Lcd_IsSramMspCbValid[Instance] = 1U;
    }
  }

  /* BSP status */
  return ret;
}

/**
  * @brief Register LCD Msp Callback registering
  * @param Callbacks pointer to LCD MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_LCD_RegisterMspCallbacks(uint32_t Instance, BSP_LCD_Cb_t *Callback)
{
  int32_t ret = BSP_ERROR_NONE;

  if (Instance >= LCD_INSTANCES_NBR) {
    ret = BSP_ERROR_WRONG_PARAM;
  } else {
    __HAL_SRAM_RESET_HANDLE_STATE(&hlcd_sram[Instance]);

    /* Register MspInit/MspDeInit Callbacks */
    if (HAL_SRAM_RegisterCallback(&hlcd_sram[Instance], HAL_SRAM_MSP_INIT_CB_ID, Callback->pMspFmcInitCb) != HAL_OK) {
      ret = BSP_ERROR_PERIPH_FAILURE;
    } else if (HAL_SRAM_RegisterCallback(&hlcd_sram[Instance], HAL_SRAM_MSP_DEINIT_CB_ID, Callback->pMspFmcDeInitCb) != HAL_OK) {
      ret = BSP_ERROR_PERIPH_FAILURE;
    } else {
      Lcd_IsSramMspCbValid[Instance] = 1U;
    }
  }

  /* BSP status */
  return ret;
}
#endif /* USE_HAL_SRAM_REGISTER_CALLBACKS */


/**
  * @brief  Probe the ST7789H2 LCD driver.
  * @param  Orientation LCD_ORIENTATION_PORTRAIT, LCD_ORIENTATION_LANDSCAPE,
  *                     LCD_ORIENTATION_PORTRAIT_ROT180 or LCD_ORIENTATION_LANDSCAPE_ROT180.
  * @retval BSP status.
  */
static int32_t ST7789H2_Probe(uint32_t Orientation)
{
  int32_t                  status;
  ST7789H2_IO_t            IOCtx;
  uint32_t                 st7789h2_id;
  static ST7789H2_Object_t ST7789H2Obj;
  uint32_t                 lcd_orientation;

  /* Configure the LCD driver */
  IOCtx.Address     = LCD_FMC_ADDRESS;
  IOCtx.Init        = LCD_FMC_Init;
  IOCtx.DeInit      = LCD_FMC_DeInit;
  IOCtx.ReadReg     = LCD_FMC_ReadReg16;
  IOCtx.WriteReg    = LCD_FMC_WriteReg16;
  IOCtx.SendData    = LCD_FMC_Send;
  IOCtx.GetTick     = LCD_FMC_GetTick;

  if (ST7789H2_RegisterBusIO(&ST7789H2Obj, &IOCtx) != ST7789H2_OK) {
    status = BSP_ERROR_BUS_FAILURE;
  } else if (ST7789H2_ReadID(&ST7789H2Obj, &st7789h2_id) != ST7789H2_OK) {
    status = BSP_ERROR_COMPONENT_FAILURE;
  } else if (st7789h2_id != ST7789H2_ID) {
    status = BSP_ERROR_UNKNOWN_COMPONENT;
  } else {
    Lcd_CompObj[0] = &ST7789H2Obj;
    Lcd_Drv[0] = (LCD_Drv_t *) &ST7789H2_Driver;
    if (Orientation == LCD_ORIENTATION_PORTRAIT) {
      lcd_orientation = ST7789H2_ORIENTATION_PORTRAIT;
    } else if (Orientation == LCD_ORIENTATION_LANDSCAPE) {
      lcd_orientation = ST7789H2_ORIENTATION_LANDSCAPE;
    } else if (Orientation == LCD_ORIENTATION_PORTRAIT_ROT180) {
      lcd_orientation = ST7789H2_ORIENTATION_PORTRAIT_ROT180;
    } else {
      lcd_orientation = ST7789H2_ORIENTATION_LANDSCAPE_ROT180;
    }
    if (Lcd_Drv[0]->Init(Lcd_CompObj[0], ST7789H2_FORMAT_RBG565, lcd_orientation) < 0) {
      status = BSP_ERROR_COMPONENT_FAILURE;
    } else {
      status = BSP_ERROR_NONE;
    }
  }

  return status;
}

/**
  * @brief  Reset ST7789H2 and activate backlight.
  * @retval None.
  */
static void ST7789H2_PowerUp(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Initialize and configure the ST7789H2 power pin */
  LCD_POWER_GPIO_CLOCK_ENABLE();
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin   = LCD_POWER_GPIO_PIN;
  HAL_GPIO_Init(LCD_POWER_GPIO_PORT, &GPIO_InitStruct);

  /* Power on the ST7789H2 */
  HAL_GPIO_WritePin(LCD_POWER_GPIO_PORT, LCD_POWER_GPIO_PIN, GPIO_PIN_RESET);

  /* Initialize and configure the ST7789H2 reset pin */
  LCD_RESET_GPIO_CLOCK_ENABLE();
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin   = LCD_RESET_GPIO_PIN;
  HAL_GPIO_Init(LCD_RESET_GPIO_PORT, &GPIO_InitStruct);

  /* Reset the ST7789H2 */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_GPIO_PIN, GPIO_PIN_RESET);
  HAL_Delay(1); /* wait at least 10us according ST7789H2 datasheet */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_GPIO_PIN, GPIO_PIN_SET);
  HAL_Delay(120); /* wait maximum 120ms according ST7789H2 datasheet */

  /* Initialize GPIO for backlight control and enable backlight */
  LCD_BACKLIGHT_GPIO_CLOCK_ENABLE();
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin       = LCD_BACKLIGHT_GPIO_PIN;
  HAL_GPIO_Init(LCD_BACKLIGHT_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_PORT, LCD_BACKLIGHT_GPIO_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Reset ST7789H2 and deactivate backlight.
  * @retval BSP status.
  */
static void ST7789H2_PowerDown(void)
{
  /* Deactivate backlight */
  HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_PORT, LCD_BACKLIGHT_GPIO_PIN, GPIO_PIN_RESET);

  /* Reset the ST7789H2 */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_GPIO_PIN, GPIO_PIN_RESET);

  /* Power down the ST7789H2 */
  HAL_GPIO_WritePin(LCD_POWER_GPIO_PORT, LCD_POWER_GPIO_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Initialize FMC.
  * @retval BSP status.
  */
static int32_t LCD_FMC_Init(void)
{
  int32_t status = BSP_ERROR_NONE;

  hlcd_sram[0].Instance    = FMC_NORSRAM_DEVICE;
  hlcd_sram[0].Extended    = FMC_NORSRAM_EXTENDED_DEVICE;
  hlcd_sram[0].Init.NSBank = FMC_NORSRAM_BANK1;

  if (HAL_SRAM_GetState(&hlcd_sram[0]) == HAL_SRAM_STATE_RESET) {
#if (USE_HAL_SRAM_REGISTER_CALLBACKS == 0)
		/* Init the FMC Msp */
		FMC_MspInit(&hlcd_sram[0]);

		if (MX_FMC_BANK1_Init(&hlcd_sram[0]) != HAL_OK) {
			status = BSP_ERROR_BUS_FAILURE;
		}
#else
		if (Lcd_IsSramMspCbValid[0] == 0U) {
			if (BSP_LCD_RegisterDefaultMspCallbacks(0) != BSP_ERROR_NONE) {
				status = BSP_ERROR_MSP_FAILURE;
			}
		}

		if (status == BSP_ERROR_NONE) {
			if (MX_FMC_BANK1_Init(&hlcd_sram[0]) != HAL_OK) {
				status = BSP_ERROR_BUS_FAILURE;
			}
		}
#endif
  }

  return status;
}

/**
  * @brief  DeInitialize BSP FMC.
  * @retval BSP status.
  */
static int32_t LCD_FMC_DeInit(void)
{
  int32_t status = BSP_ERROR_NONE;

#if (USE_HAL_SRAM_REGISTER_CALLBACKS == 0)
  FMC_MspDeInit(&hlcd_sram[0]);
#endif

  /* De-Init the FMC */
  if (HAL_SRAM_DeInit(&hlcd_sram[0]) != HAL_OK) {
    status = BSP_ERROR_PERIPH_FAILURE;
  }

  return status;
}

/**
  * @brief  Write 16bit values in registers of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg     The target register start address to write.
  * @param  pData   Pointer to data buffer.
  * @param  Length  Number of data.
  * @retval BSP status.
  */
static int32_t LCD_FMC_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint32_t Length)
{
  int32_t  ret = BSP_ERROR_NONE;
  uint16_t i = 0;

  if ((DevAddr != LCD_FMC_ADDRESS) || (pData == NULL) || (Length == 0U)) {
    ret = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Write register address */
    *(uint16_t *)LCD_REGISTER_ADDR = Reg;
    while (i < (2U * Length)) {
      /* Write register value */
      *(uint16_t *)LCD_DATA_ADDR = (((uint16_t)pData[i + 1U] << 8U) & 0xFF00U) | ((uint16_t)pData[i] & 0x00FFU);
      /* Update data pointer */
      i += 2U;
    }
  }

  /* BSP status */
  return ret;
}

/**
  * @brief  Read 16bit values in registers of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg     The target register start address to read.
  * @param  pData   Pointer to data buffer.
  * @param  Length  Number of data.
  * @retval BSP status.
  */
static int32_t LCD_FMC_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint32_t Length)
{
  int32_t  ret = BSP_ERROR_NONE;
  uint16_t i = 0;
  uint16_t tmp;

  if ((DevAddr != LCD_FMC_ADDRESS) || (pData == NULL) || (Length == 0U)) {
    ret = BSP_ERROR_WRONG_PARAM;
  } else {
    /* Write register address */
    *(uint16_t *)LCD_REGISTER_ADDR = Reg;
    while (i < (2U * Length)) {
      tmp = *(uint16_t *)LCD_DATA_ADDR;
      pData[i]    = (uint8_t) tmp;
      pData[i + 1U] = (uint8_t)(tmp >> 8U);
      /* Update data pointer */
      i += 2U;
    }
  }

  /* BSP status */
  return ret;
}

/**
  * @brief  Send 16bit values to device through BUS.
  * @param  pData   Pointer to data buffer.
  * @param  Length  Number of data.
  * @retval BSP status.
  */
static int32_t LCD_FMC_Send(uint8_t *pData, uint32_t Length)
{
  int32_t  ret = BSP_ERROR_NONE;
  uint16_t i = 0;

  if ((pData == NULL) || (Length == 0U)) {
    ret = BSP_ERROR_WRONG_PARAM;
  } else {
    while (i < (2U * Length)) {
      /* Send value */
      *(uint16_t *)LCD_REGISTER_ADDR = (((uint16_t)pData[i + 1U] << 8U) & 0xFF00U) | ((uint16_t)pData[i] & 0x00FFU);
      /* Update data pointer */
      i += 2U;
    }
  }

  /* BSP status */
  return ret;
}

/**
  * @brief  Provide a tick value in millisecond.
  * @retval Tick value.
  */
static int32_t LCD_FMC_GetTick(void)
{
  uint32_t ret;
  ret = HAL_GetTick();
  return (int32_t)ret;
}

/**
  * @brief  Initializes FMC MSP.
  * @param  hSram : SRAM handler
  * @retval None
  */
static void  FMC_MspInit(SRAM_HandleTypeDef *hSram)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(hSram);

  /*** Configure the GPIOs ***/

  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* Common GPIO configuration */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init_structure.Alternate = GPIO_AF12_FMC;
  gpio_init_structure.Pull      = GPIO_PULLUP;

  /*## NE configuration #######*/
  /* NE1 : LCD */
  gpio_init_structure.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /*## NOE and NWE configuration #######*/
  gpio_init_structure.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /*## RS configuration #######*/
  gpio_init_structure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOF, &gpio_init_structure);

  /*## Data Bus #######*/
  gpio_init_structure.Pull      = GPIO_NOPULL;
  /* GPIOD configuration */
  gpio_init_structure.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_8 | GPIO_PIN_9 | \
                            GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /* GPIOE configuration */
  gpio_init_structure.Pin = GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 | \
                            GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | \
                            GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &gpio_init_structure);
}

/**
  * @brief  DeInitializes FMC MSP.
  * @param  hSram : SRAM handler
  * @retval None
  */
static void FMC_MspDeInit(SRAM_HandleTypeDef *hSram)
{
  GPIO_InitTypeDef gpio_init_structure;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(hSram);

  /* GPIOD configuration */
  gpio_init_structure.Pin   = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_4  | GPIO_PIN_5  | \
                              GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 | \
                              GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_DeInit(GPIOD, gpio_init_structure.Pin);

  /* GPIOE configuration */
  gpio_init_structure.Pin   = GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 | \
                              GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | \
                              GPIO_PIN_15;
  \

  HAL_GPIO_DeInit(GPIOE, gpio_init_structure.Pin);

  /* GPIOF configuration */
  gpio_init_structure.Pin   = GPIO_PIN_0;
  HAL_GPIO_DeInit(GPIOF, gpio_init_structure.Pin);

  /* Disable FMC clock */
  __HAL_RCC_FMC_CLK_DISABLE();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
