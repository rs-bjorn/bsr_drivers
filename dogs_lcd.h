/**
  ******************************************************************************
  * @file           : dogslcd.h
  * @author			: bjornhropot
  * @date			: Nov 17, 2022
  * @brief          : 
  ******************************************************************************
  * @attention
  *
  *
  * @todo
  * TODO
  ******************************************************************************
*/
#ifndef INC_DOGS_LCD_H_
#define INC_DOGS_LCD_H_
//INCLUDES ***********************************************************************************************
//includes and externs
#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

//SETUP **************************************************************************************************
extern SPI_HandleTypeDef hspi1;

//#define USE_RTOS
#define DEF_DOGS_MOSI_PORT                      DOGS_MOSI_GPIO_Port
#define DEF_DOGS_MOSI_PIN                       DOGS_MOSI_Pin
#define DEF_DOGS_RST_PORT                       DOGS_RST_GPIO_Port
#define DEF_DOGS_RST_PIN                        DOGS_RST_Pin
#define DEF_DOGS_CD_PORT                        DOGS_CD_GPIO_Port
#define DEF_DOGS_CD_PIN                         DOGS_CD_Pin
#define DEF_DOGS_RST_PORT                       DOGS_RST_GPIO_Port
#define DEF_DOGS_RST_PIN                        DOGS_RST_Pin
#define DEF_DOGS_SCK_PORT                       DOGS_SCK_GPIO_Port
#define DEF_DOGS_SCK_PIN                        DOGS_SCK_Pin
#define DEF_DOGS_CS0_PORT                       DOGS_CS0_GPIO_Port
#define DEF_DOGS_CS0_PIN                        DOGS_CS0_Pin

#define DOGS102 4
#define DISPLAY_WIDTH 102

#define ALIGN_LEFT 1
#define ALIGN_RIGHT 2
#define ALIGN_CENTER 3

#define STYLE_NORMAL 1
#define STYLE_FULL 2
#define STYLE_INVERSE 3
#define STYLE_FULL_INVERSE 4

#define VIEW_BOTTOM 0xC0
#define VIEW_TOP 0xC8

//CONFIG *************************************************************************************************

#ifdef USE_RTOS
#define Delay(x) HAL_Delay(x)
#else
#define Delay(x) osDelay(x)
#endif

//LOGIC **************************************************************************************************
void dogs_spi_put_byte(uint8_t dat);

void dogs_spi_put(uint8_t *dat, int len);

void dogs_spi_out(uint8_t dat);

void dogs_command(uint8_t dat);

void dogs_data(uint8_t dat);

void dogs_position(uint8_t column, uint8_t page);

void dogs_begin(void);

void dogs_clear(void);

void dogs_contrast(uint8_t contr);

void dogs_view(uint8_t direction);

void dogs_all_pixel_on(bool state);

void dogs_inverse(bool state);

void dogs_sleep(bool state);

void dogs_string(int column, uint8_t page, const uint8_t *font_adress, const char *str, uint8_t align, uint8_t style);

void dogs_picture(uint8_t column, uint8_t page, const uint8_t *pic_adress);

#endif /* INC_DOGS_LCD_H_ */
