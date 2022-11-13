/**
 *********************************************************************************************************
 * @file            : rslora.h
 * @author		      : bjornhropot
 * @date			      : 14/11/2022
 * @brief           : Ported LoRa library for RFM95
 *********************************************************************************************************
 * @attention
 * Start setup by checking initialisation function, as well as all cases of GPIO ports and Pins.
 * Check whether or not RTOS is in use (to prevent HAL_Delay).
 *
 * @todo
 * TODO
 *********************************************************************************************************
 */
#ifndef INC_RSLORA_H_
#define INC_RSLORA_H_

//INCLUDES ***********************************************************************************************
//includes and externs
#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

//SETUP **************************************************************************************************
extern SPI_HandleTypeDef hspi1;

//#define USE_RTOS
#define DEF_LORA_NSS_GPIO_Port                  LORA_NSS_GPIO_Port
#define DEF_LORA_NSS_Pin                        LORA_NSS_Pin
#define OP_FREQUENCY                            915E6

//CONFIG *************************************************************************************************

#ifdef USE_RTOS
#define Delay(x) HAL_Delay(x)
#else
#define Delay(x) osDelay(x)
#endif

// Enumeration
typedef enum
{
  LORA_OK = 0x00, LORA_ERROR = 0x01, LORA_VERSION_ERROR = 0x02, LORA_TIMEOUT = 0x03
} lora_status_t;

extern int _packetIndex;
//LOGIC **************************************************************************************************
// Function Prototypes
/*
 * @brief:  this function reads a register from the lora module
 * @args:   register_address using enumerated value
 *          return_val pointer to uint8
 *
 * @return: returns lora status
 */
lora_status_t lora_spi_transfer(uint8_t register_addr, uint8_t register_set, uint8_t *return_val);

/*
 * @brief:  this function allows registers from the LoRa chip to be read
 * @args:   register address to read
 *
 * @return: unsigned integer of register contents
 */
uint8_t lora_read_register(uint8_t address);

/*
 * @brief:  this function allows registers on the LoRa chip to be written
 * @args:   register address to write to
 *          value to write into the register
 */
void lora_write_register(uint8_t address, uint8_t value);

/*
 * @brief:  initialises an RFM95 module to start communicating
 */
lora_status_t lora_initialise();

//ARDUINO LIB ********************************************************************************************
//following functions read/write registers to LoRa module to achieve outcome. Original library did not
//include any comments.

void lora_sleep();

void lora_idle();

void lora_set_frequency(long frequency);

void lora_set_ocp(uint8_t mA);

void lora_set_tx_power(int level, int outputPin);

uint8_t lora_random();

void lora_dump_registers();

void lora_end();

void lora_explicit_header_mode();

void lora_implicit_header_mode();

bool lora_is_transmitting();

int lora_begin_packet(int implicitHeader);

int lora_end_packet(bool async);

int lora_parse_packet(int size);

int lora_packet_rssi();

int lora_packet_snr();

int lora_rssi();

size_t lora_write(const uint8_t *buffer, size_t size);

int lora_available();

int lora_read();

int lora_peek();

void lora_receive(int size);

long lora_get_signal_bandwidth();

int lora_get_spreading_factor();

void lora_set_ldo_flag();

void lora_set_spreading_factor(int sf);

void lora_set_signal_bandwidth(long sbw);

void lora_set_coding_rate_4(int denominator);

void lora_set_preamble_length(long length);

void lora_set_sync_word(int sw);

void lora_enable_crc();

void lora_disable_crc();

void lora_enable_invert_iq();

void lora_disable_invert_iq();

void lora_set_gain(uint8_t gain);

#endif /* INC_RSLORA_H_ */
//EOF ****************************************************************************************************
