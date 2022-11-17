/**
  ********************************************************************************************************
  * @file           : dogslcd.c
  * @author			: bjornhropot
  * @date			: Nov 17, 2022
  * @brief          : 
  ********************************************************************************************************
  * @attention
  *
  *
  * @todo
  * TODO
  ********************************************************************************************************
*/
//CONFIG *************************************************************************************************
//Includes
#include "dogs_lcd.h"

//Definitions and array for initalisation sequence
#define INITLEN_DOGS102 13
uint8_t init_dogs102[INITLEN_DOGS102] = {0x40, 0xA1, 0xC0, 0xA4, 0x82, 0x82, 0x2F, 0x07, 0x81, 0x0C, 0xF8, 0x00, 0xAF};
//uint8_t init_dogs102[INITLEN_DOGS102] = {0x40, 0xA1, 0xC0, 0xA4, 0xA6, 0xA2, 0x2F, 0x27, 0x81, 0x10, 0xFA, 0x90, 0xAF};


//Arduino Port *******************************************************************************************
//super globals
uint8_t type;
bool hardware;
bool top_view;

uint8_t *canvas;

uint8_t drawMode;
uint8_t canvasSizeX;
uint8_t canvasSizeY;
uint8_t canvasPages;

int canvasUpperLeftX;
int canvasUpperLeftY;


// Functions below


/*----------------------------
Func: spi_put_byte
Desc: Sends one Byte using CS
Vars: data
------------------------------*/
void dogs_spi_put_byte(uint8_t dat)
{
  uint8_t tx_buff[1] = {dat};
  HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx_buff, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_SET);
}

/*----------------------------
Func: spi_put
Desc: Sends bytes using CS
Vars: ptr to data and len
------------------------------*/
void dogs_spi_put(uint8_t *dat, int len)
{
  HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_RESET);
  do
  {
    dogs_spi_out(*dat++);
  }while(--len);


  HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_SET);
}

/*----------------------------
Func: spi_out
Desc: Sends one Byte, no CS
Vars: data
------------------------------*/
void dogs_spi_out(uint8_t dat)
{
  uint8_t i = 8;
  uint8_t tx_buff[1] = {dat};
//  printf("Transmitting 0x%02X\r\n", tx_buff[0]);
  HAL_SPI_Transmit(&hspi1, tx_buff, 1, HAL_MAX_DELAY);
}

/*----------------------------
Func: command
Desc: Sends a command to the DOG-Display
Vars: data
------------------------------*/
void dogs_command(uint8_t dat)
{
  HAL_GPIO_WritePin(DEF_DOGS_CD_PORT,DEF_DOGS_CD_PIN,GPIO_PIN_RESET);
  dogs_spi_put_byte(dat);
}

/*----------------------------
Func: data
Desc: Sends data to the DOG-Display
Vars: data
------------------------------*/
void dogs_data(uint8_t dat)
{
  HAL_GPIO_WritePin(DEF_DOGS_CD_PORT,DEF_DOGS_CD_PIN,GPIO_PIN_SET);
  dogs_spi_put_byte(dat);
}

/*----------------------------
Func: position
Desc: sets write pointer in DOG-Display
Vars: column (0..127/131), page(0..3/7)
------------------------------*/
void dogs_position(uint8_t column, uint8_t page)
{
  //column += 4;

  dogs_command(0x10 + (column>>4)); //MSB address column
  dogs_command(0x00 + (column&0x0F)); //LSB address column
  dogs_command(0xB0 + (page&0x0F)); //address page
}

/*----------------------------
Func: Arduino begin function with Hardware SPI
Desc: Initializes SPI Hardware and DOG Displays
Vars: Spi-Port, CS-Pin, A0-Pin (high=data, low=command), p_res = Reset-Pin, type (1=EA DOGM128-6, 2=EA DOGL128-6)
------------------------------*/
void dogs_begin(void)
{
  uint8_t *ptr_init; //pointer to the correct init values

  top_view = false; //default = bottom view

  //perform a Reset
  HAL_GPIO_WritePin(DEF_DOGS_RST_PORT,DEF_DOGS_RST_PIN,GPIO_PIN_RESET);
  Delay(1);    //should be 10us
  HAL_GPIO_WritePin(DEF_DOGS_RST_PORT,DEF_DOGS_RST_PIN,GPIO_PIN_SET);
  Delay(1);

  //Init DOGM displays, depending on users choice
  ptr_init = init_dogs102;

  type = 4;

  HAL_GPIO_WritePin(DEF_DOGS_CD_PORT,DEF_DOGS_CD_PIN,GPIO_PIN_RESET); //init the display
  dogs_spi_put(ptr_init, INITLEN_DOGS102);  // shorter init for DOGS102

  dogs_clear();
}

/*----------------------------
Func: clear_display
Desc: clears the entire DOG-Display
Vars: ---
------------------------------*/
void dogs_clear(void)
{
  uint8_t page = 0;
  uint8_t column = 0;
  uint8_t page_cnt = 8;

  for(page = 0; page < page_cnt; page++) //Display has 8 pages
  {
    dogs_position(0,page);
    HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DEF_DOGS_CD_PORT,DEF_DOGS_CD_PIN,GPIO_PIN_SET);

    for(column = 0; column < DISPLAY_WIDTH; column++){ //clear the whole page line
      dogs_spi_out(0x00);
    }

    HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_SET);
  }
}

/*----------------------------
Func: contrast
Desc: sets contrast to the DOG-Display
Vars: byte contrast (0..63)
------------------------------*/
void dogs_contrast(uint8_t contr)
{
  dogs_command(0x81);  //double byte command
  dogs_command(contr&0x3F);  //contrast has only 6 bits
}


/*----------------------------
Func: view
Desc: ssets the display viewing direction
Vars: direction (top view 0xC8, bottom view (default) = 0xC0)
------------------------------*/
void dogs_view(uint8_t direction)
{
  if(direction == VIEW_TOP)
  {
    top_view = true;
    dogs_command(0xA0);
  }
  else
  {
    top_view = false;
    dogs_command(0xA1);
  }

  dogs_command(direction);

  dogs_clear(); //Clear screen, as old content is not usable (mirrored)
}

/*----------------------------
Func: all_pixel_on
Desc: sets all pixel of the display to on
Vars: state (false=show SRAM content, true=set all pixel to on)
------------------------------*/
void dogs_all_pixel_on(bool state)
{
  if(state == false)
  {
    dogs_command(0xA4);  // normal mode
  }
  else
  {
    dogs_command(0xA5);  // all pixels on
  }
}

/*----------------------------
Func: inverse
Desc: inverse content of display
Vars: state (false=normal content, true=inverse content)
------------------------------*/
void dogs_inverse(bool state)
{
  if(state == false)
  {
    dogs_command(0xA6);  // normal mode
  }
  else
  {
    dogs_command(0xA7);  // inverse mode
  }
}

/*----------------------------
Func: sleep
Desc: sends the display to sleep mode (on/off)
Vars: state (false=normal mode, true=sleep mode)
------------------------------*/
void dogs_sleep(bool state)
{
  if(state == false)
  {
    dogs_command(0xAF);  // normal mode
  }
  else
  {
    dogs_command(0xAE);  // sleep mode
  }
}

//Graphic Functions **************************************************************************************
/*----------------------------
Func: string
Desc: shows string with selected font on position with align and style
Vars: column (0..127/131), page(0..3/7),  font address in program memory, stringarray, align, style
------------------------------*/
void dogs_string(int column, uint8_t page, const uint8_t *font_adress, const char *str, uint8_t align, uint8_t style)
{
  unsigned int pos_array;  //Position of character data in memory array
  uint8_t x, y, width_max,width_min;  //temporary column and page address, couloumn_cnt tand width_max are used to stay inside display area
  int column_cnt;  //temporary column and page address, couloumn_cnt tand width_max are used to stay inside display area
  uint8_t start_code, last_code, width, page_height, bytes_p_char; //font information, needed for calculation
  const char *string;
  int stringwidth=0; // width of string in pixels

  start_code = font_adress[2];  //get first defined character
  last_code = font_adress[3];  //get last defined character
  width = font_adress[4];  //width in pixel of one char
  page_height = font_adress[6];  //page count per char
  bytes_p_char = font_adress[7];  //bytes per char

  string = str;  //temporary pointer to the beginning of the string to print
  while(*string != 0)
  {
    if((uint8_t)*string < start_code || (uint8_t)*string > last_code) //make sure data is valid
      string++;
    else
    {
      string++;
      stringwidth++;
    }
  }
  stringwidth*=width;

  if(page_height + page > 8){ //stay inside display area
    page_height = 8 - page;
  }

  if(align==ALIGN_RIGHT)
  {
    if(column==0) column=DISPLAY_WIDTH-stringwidth;  //if column is 0 align string to the right border
    else column=column-stringwidth;
  }
  if(align==ALIGN_CENTER) column=(DISPLAY_WIDTH-stringwidth)/2;

  //The string is displayed character after character. If the font has more then one page,
  //the top page is printed first, then the next page and so on
  for(y = 0; y < page_height; y++)
  {
    if(style==STYLE_FULL || style==STYLE_FULL_INVERSE)
    {
      dogs_position(0, page+y); //set startpositon and page
      column_cnt=0;
      HAL_GPIO_WritePin(DEF_DOGS_CD_PORT,DEF_DOGS_CD_PIN,GPIO_PIN_SET);
      HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_RESET);
      while(column_cnt<column)  // fill columns until beginning of string
      {
        column_cnt++;
        if(style==STYLE_FULL_INVERSE){
          dogs_spi_out(0xFF);
        }
        else{
          dogs_spi_out(0x00);
        }
      }
    }
    else if(column<0) {
      dogs_position(0,page+y);
    }
    else {
      dogs_position(column, page+y); //set startpositon and page
    }
    column_cnt = column; //store column for display last column check
    string = str; //temporary pointer to the beginning of the string to print
    HAL_GPIO_WritePin(DEF_DOGS_CD_PORT,DEF_DOGS_CD_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_RESET);
    while(*string != 0)
    {
      if(column_cnt>DISPLAY_WIDTH) string++;
      else if(column_cnt+width<0)
      {
        string++;
        column_cnt+=width;
      }
      else if((uint8_t)*string < start_code || (uint8_t)*string > last_code) //make sure data is valid
        string++;
      else
      {
        //calculate position of ascii character in font array
        //bytes for header + (ascii - startcode) * bytes per char)
        pos_array = 8 + (unsigned int)(*string++ - start_code) * bytes_p_char;
        pos_array += y*width; //get the dot pattern for the part of the char to print

        if((column_cnt + width) > DISPLAY_WIDTH) //stay inside display area
          width_max = DISPLAY_WIDTH-column_cnt;
        else
          width_max = width;

        if(column_cnt<0) width_min=0-column_cnt;
        else width_min=0;

        for(x=width_min; x < width_max; x++) //print the whole string
        {

          if(style==STYLE_INVERSE || style==STYLE_FULL_INVERSE) {
            dogs_spi_out(~font_adress[pos_array+x]);
          }
          else {
            dogs_spi_out(font_adress[pos_array+x]);
          }
          //spi_out(pgm_read_byte(&font_adress[pos_array+x])); //double width font (bold)
        }
        column_cnt+=width;
      }
    }
    if(style==STYLE_FULL || style==STYLE_FULL_INVERSE)
    {
      column_cnt=column+stringwidth;
      while(column_cnt<DISPLAY_WIDTH)
      {
        column_cnt++;
        if(style==STYLE_FULL_INVERSE) {
          dogs_spi_out(0xFF);
        }
        else {
          dogs_spi_out(0);
        }
      }
    }
    HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_SET);
  }
}

/*----------------------------
Func: rectangle
Desc: shows a pattern filled rectangle on the display
Vars: start and end column (0..127/131) and page(0..3/7), bit pattern
------------------------------*/
void dogs_rectangle(uint8_t start_column, uint8_t start_page, uint8_t end_column, uint8_t end_page, uint8_t pattern)
{
  uint8_t x;
  uint8_t y;

  if(end_column>DISPLAY_WIDTH){  //stay inside display area
    end_column=DISPLAY_WIDTH;
  }

  end_page = 7;

  for(y=start_page; y<=end_page; y++)
  {
    position(start_column, y);
    HAL_GPIO_WritePin(DEF_DOGS_CD_PORT,DEF_DOGS_CD_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_RESET);

    for(x=start_column; x<=end_column; x++){
      spi_out(pattern);
    }

    HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_SET);
  }
}

/*----------------------------
Func: picture
Desc: shows a BLH-picture on the display (see BitMapEdit EA LCD-Tools (http://www.lcd-module.de/support.html))
Vars: column (0..101) and page(0..7), program memory adress of data
------------------------------*/
void dogs_picture(uint8_t column, uint8_t page, const uint8_t *pic_adress)
{
    uint8_t c;
    uint8_t p;
    unsigned int byte_cnt = 2;
    uint8_t width;
    uint8_t page_cnt;


    width = pic_adress[0];
    page_cnt = (pic_adress[1] + 7) / 8; //height in pages, add 7 and divide by 8 for getting the used pages (byte boundaries)

    if(width + column > 102) //stay inside display area
        width = 102 - column;
    if(page_cnt + page > 8)
        page_cnt = 8 - page;

    for(p=0; p<page_cnt; p++)
    {
        dogs_position(column, page + p);
        HAL_GPIO_WritePin(DEF_DOGS_CD_PORT,DEF_DOGS_CD_PIN,GPIO_PIN_SET);
        HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_RESET);

        for(c=0; c<width; c++){
            dogs_spi_out(pic_adress[byte_cnt++]);
        }

        HAL_GPIO_WritePin(DEF_DOGS_CS0_PORT,DEF_DOGS_CS0_PIN,GPIO_PIN_SET);
    }
}
