/**
 ********************************************************************************************************
 * @file            : rslora.c
 * @author			    : bjornhropot
 * @date			      : 14/11/2022
 * @brief           : Ported LoRa library for RFM95
 ********************************************************************************************************
 * @attention
 *
 *
 * @todo
 * TODO
 * please review all comments marked as ATTENTION (ctrl+f and fix)
 ********************************************************************************************************
 */

//CONFIG *************************************************************************************************
// includes
#include "rslora.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255
#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

//variables as defined in lora library
int _ss;
int _reset;
int _dio0;
long _frequency;
int _packetIndex = 0;
int _implicitHeaderMode;

//LOGIC **************************************************************************************************

// Read register from LORA module
lora_status_t spi_transfer(uint8_t register_addr, uint8_t register_set, uint8_t *return_val)
{
  HAL_StatusTypeDef hal_status;
  uint8_t tx_data[2];
  uint8_t rx_data[2];

  //define transmit and receive buffers
  tx_data[0] = register_addr;                // read operation
  tx_data[1] = register_set;                 // byte to write. 0x00 to read

  //set NSS pin to low and perform transmit receive
  HAL_GPIO_WritePin(DEF_LORA_NSS_GPIO_Port, DEF_LORA_NSS_Pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(DEF_LORA_NSS_GPIO_Port, DEF_LORA_NSS_Pin, GPIO_PIN_SET);
//  printf("TX 0x%02x 0x%02x | RX 0x%02x 0x%02x\r\n", tx_data[0], tx_data[1], rx_data[0], rx_data[1]);

  if (hal_status == HAL_OK)
  {
    // response is in the second byte (first byte is null)
    *return_val = rx_data[1];
  }
  else
  {
    //general SPI error
    return LORA_TIMEOUT;
  }

  return LORA_OK;
}

// Register read for arduino library
uint8_t lora_read_register(uint8_t address)
{
  uint8_t rx_buff;

  //perform a lora read operation
  if (spi_transfer(address & 0x7f, 0x00, &rx_buff) != LORA_OK)
  {
    //error with single transfer return 0x00
    return 0x00;
  }

  //all ok, return value
  return rx_buff;
}

// Register write for arduino library
void lora_write_register(uint8_t address, uint8_t value)
{
  uint8_t rx_buff;

  //perform a lora read operation
  if (spi_transfer(address | 0x80, value, &rx_buff) != LORA_OK)
  {
    //catch error here

  }
}

//ARDUINO LIB*********************************************************************************************
/*
 * Lack of comments is the fault of original author
 *
 */
void lora_sleep()
{
  lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void lora_idle()
{
  lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void lora_set_frequency(long frequency)
{
  uint64_t frf = ((uint64_t) frequency << 19) / 32000000;

  lora_write_register(REG_FRF_MSB, (uint8_t) (frf >> 16));
  lora_write_register(REG_FRF_MID, (uint8_t) (frf >> 8));
  lora_write_register(REG_FRF_LSB, (uint8_t) (frf >> 0));
}

void lora_set_ocp(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120)
  {
    ocpTrim = (mA - 45) / 5;
  }
  else if (mA <= 240)
  {
    ocpTrim = (mA + 30) / 10;
  }

  lora_write_register(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void lora_set_tx_power(int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin)
  {
    // RFO
    if (level < 0)
    {
      level = 0;
    }
    else if (level > 14)
    {
      level = 14;
    }

    lora_write_register(REG_PA_CONFIG, 0x70 | level);
  }
  else
  {
    // PA BOOST
    if (level > 17)
    {
      if (level > 20)
      {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      lora_write_register(REG_PA_DAC, 0x87);
      lora_set_ocp(140);
    }
    else
    {
      if (level < 2)
      {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      lora_write_register(REG_PA_DAC, 0x84);
      lora_set_ocp(100);
    }

    lora_write_register(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

uint8_t lora_random()
{
  return lora_read_register(REG_RSSI_WIDEBAND);
}

// Initialise the LORA module
lora_status_t lora_initialise()
{
  uint8_t rx_buff;

  // start by performing a hardware reset
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
  Delay(10);
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
  Delay(10);

  // quert module for lora version
  if (spi_transfer(REG_VERSION, 0x00, &rx_buff) != LORA_OK)
  {
    //catch error reading register
    printf("ERROR: LORA READ REGISTER FAILED!\r\n");
    return LORA_ERROR;
  }

  // check if lora version matches current version
  if (rx_buff != 0x12)
  {
    // catch error in version mismatch
    printf("ERROR: LORA VERSION INCORRECT! MODULE VERSION 0x%02x \r\n", rx_buff);
    return LORA_VERSION_ERROR;
  }

  // all ok, continue
  printf("INFO: LORA MODULE READY USING VERSION 0x%02x \r\n", rx_buff);

  // put in lora_sleep mode
  lora_sleep();

  // set frequency
  lora_set_frequency(OP_FREQUENCY);

  // set base addresses
  lora_write_register(REG_FIFO_TX_BASE_ADDR, 0);
  lora_write_register(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  lora_write_register(REG_LNA, lora_read_register(REG_LNA) | 0x03);

  // set auto AGC
  lora_write_register(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  lora_set_tx_power(20, PA_BOOST);

  // put in standby mode
  lora_idle();

  return LORA_OK;
}

void lora_dump_registers()
{
  for (int i = 0; i < 128; i++)
  {
    printf("REGISTER: 0x%02x\r\n", lora_read_register(i));
  }
}

void lora_end()
{
  // put in lora_sleep mode
  lora_sleep();

  // stop SPI
  //call hal spi halt (might not be best choice)
  if (HAL_SPI_DeInit(&hspi1) != HAL_OK)
  {
    //handle the error
  }
}

void lora_explicit_header_mode()
{
  _implicitHeaderMode = 0;

  lora_write_register(REG_MODEM_CONFIG_1, lora_read_register(REG_MODEM_CONFIG_1) & 0xfe);
}

void lora_implicit_header_mode()
{
  _implicitHeaderMode = 1;

  lora_write_register(REG_MODEM_CONFIG_1, lora_read_register(REG_MODEM_CONFIG_1) | 0x01);
}

bool lora_is_transmitting()
{
  if ((lora_read_register(REG_OP_MODE) & MODE_TX) == MODE_TX)
  {
    return true;
  }

  if (lora_read_register(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)
  {
    // clear IRQ's
    lora_write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

int lora_begin_packet(int implicitHeader)
{
  if (lora_is_transmitting())
  {
    return 0;
  }

  // put in standby mode
  lora_idle();

//  if (implicitHeader) {
//    lora_implicit_header_mode();
//  } else {
//    lora_explicit_header_mode();
//  }

  //implicit headers does not quite work. (review above code and fix accordingly)
  lora_explicit_header_mode();

  // reset FIFO address and paload length
  lora_write_register(REG_FIFO_ADDR_PTR, 0);
  lora_write_register(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int lora_end_packet(bool async)
{

  if ((async) /*&& (_onTxDone)*/)
    lora_write_register(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE

  // put in TX mode
  lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  if (!async)
  {
    // wait for TX done
    while ((lora_read_register(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
    {
      //yield();
      //some sort of osYield function needed

      /* ATTENTION */
    }
    // clear IRQ's
    lora_write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }
  return 1;
}

int lora_parse_packet(int size)
{
  int packetLength = 0;
  int irqFlags = lora_read_register(REG_IRQ_FLAGS);

  if (size > 0)
  {
    lora_implicit_header_mode();

    lora_write_register(REG_PAYLOAD_LENGTH, size & 0xff);
  }
  else
  {
    lora_explicit_header_mode();
  }

  // clear IRQ's
  lora_write_register(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
  {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode)
    {
      packetLength = lora_read_register(REG_PAYLOAD_LENGTH);
    }
    else
    {
      packetLength = lora_read_register(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    lora_write_register(REG_FIFO_ADDR_PTR, lora_read_register(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    lora_idle();
  }
  else if (lora_read_register(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
  {
    // not currently in RX mode

    // reset FIFO address
    lora_write_register(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

int lora_packet_rssi()
{
  return (lora_read_register(REG_PKT_RSSI_VALUE)
      - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

int lora_packet_snr()
{
  return ((int8_t) lora_read_register(REG_PKT_SNR_VALUE)) * 0.25;
}

int lora_rssi()
{
  return (lora_read_register(REG_RSSI_VALUE)
      - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

size_t lora_write(const uint8_t *buffer, size_t size)
{
  int currentLength = lora_read_register(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH)
  {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++)
  {
    lora_write_register(REG_FIFO, buffer[i]);
  }

  // update length
  lora_write_register(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

int lora_available()
{
  return (lora_read_register(REG_RX_NB_BYTES) - _packetIndex);
}

int lora_read()
{
  if (!lora_available())
  {
    return -1;
  }

  _packetIndex++;

  return lora_read_register(REG_FIFO);
}

int lora_peek()
{
  if (!lora_available())
  {
    return -1;
  }

  // store current FIFO address
  int currentAddress = lora_read_register(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = lora_read_register(REG_FIFO);

  // restore FIFO address
  lora_write_register(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

void lora_receive(int size)
{

  lora_write_register(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

  if (size > 0)
  {
    lora_implicit_header_mode();

    lora_write_register(REG_PAYLOAD_LENGTH, size & 0xff);
  }
  else
  {
    lora_explicit_header_mode();
  }

  lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

long lora_get_signal_bandwidth()
{
  uint8_t bw = (lora_read_register(REG_MODEM_CONFIG_1) >> 4);

  switch (bw)
  {
    case 0:
      return 7.8E3;
    case 1:
      return 10.4E3;
    case 2:
      return 15.6E3;
    case 3:
      return 20.8E3;
    case 4:
      return 31.25E3;
    case 5:
      return 41.7E3;
    case 6:
      return 62.5E3;
    case 7:
      return 125E3;
    case 8:
      return 250E3;
    case 9:
      return 500E3;
  }

  return -1;
}

int lora_get_spreading_factor()
{
  return lora_read_register(REG_MODEM_CONFIG_2) >> 4;
}

void lora_set_ldo_flag()
{
  /* ATTENTION */
  //function is kinda cursed
  // Section 4.1.1.5
  long symbolDuration = 1000 / (lora_get_signal_bandwidth() / (1L << lora_get_spreading_factor()));

  // Section 4.1.1.6
  bool ldoOn = symbolDuration > 16;

  uint8_t config3 = lora_read_register(REG_MODEM_CONFIG_3);

// replaced arduino bitwrite with c syntax
//  bitWrite(config3, 3, ldoOn);
  if (ldoOn == 1)
  {
    config3 |= ldoOn << 3;
  }
  else
  {
    config3 &= ldoOn << 3;
  }

  lora_write_register(REG_MODEM_CONFIG_3, config3);
}

void lora_set_spreading_factor(int sf)
{
  if (sf < 6)
  {
    sf = 6;
  }
  else if (sf > 12)
  {
    sf = 12;
  }

  if (sf == 6)
  {
    lora_write_register(REG_DETECTION_OPTIMIZE, 0xc5);
    lora_write_register(REG_DETECTION_THRESHOLD, 0x0c);
  }
  else
  {
    lora_write_register(REG_DETECTION_OPTIMIZE, 0xc3);
    lora_write_register(REG_DETECTION_THRESHOLD, 0x0a);
  }

  lora_write_register(REG_MODEM_CONFIG_2, (lora_read_register(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  lora_set_ldo_flag();
}

void lora_set_signal_bandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3)
  {
    bw = 0;
  }
  else if (sbw <= 10.4E3)
  {
    bw = 1;
  }
  else if (sbw <= 15.6E3)
  {
    bw = 2;
  }
  else if (sbw <= 20.8E3)
  {
    bw = 3;
  }
  else if (sbw <= 31.25E3)
  {
    bw = 4;
  }
  else if (sbw <= 41.7E3)
  {
    bw = 5;
  }
  else if (sbw <= 62.5E3)
  {
    bw = 6;
  }
  else if (sbw <= 125E3)
  {
    bw = 7;
  }
  else if (sbw <= 250E3)
  {
    bw = 8;
  }
  else /*if (sbw <= 250E3)*/
  {
    bw = 9;
  }

  lora_write_register(REG_MODEM_CONFIG_1, (lora_read_register(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  lora_set_ldo_flag();
}

void lora_set_coding_rate_4(int denominator)
{
  if (denominator < 5)
  {
    denominator = 5;
  }
  else if (denominator > 8)
  {
    denominator = 8;
  }

  int cr = denominator - 4;

  lora_write_register(REG_MODEM_CONFIG_1, (lora_read_register(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void lora_set_preamble_length(long length)
{
  lora_write_register(REG_PREAMBLE_MSB, (uint8_t) (length >> 8));
  lora_write_register(REG_PREAMBLE_LSB, (uint8_t) (length >> 0));
}

void lora_set_sync_word(int sw)
{
  lora_write_register(REG_SYNC_WORD, sw);
}

void lora_enable_crc()
{
  lora_write_register(REG_MODEM_CONFIG_2, lora_read_register(REG_MODEM_CONFIG_2) | 0x04);
}

void lora_disable_crc()
{
  lora_write_register(REG_MODEM_CONFIG_2, lora_read_register(REG_MODEM_CONFIG_2) & 0xfb);
}

void lora_enable_invert_iq()
{
  lora_write_register(REG_INVERTIQ, 0x66);
  lora_write_register(REG_INVERTIQ2, 0x19);
}

void lora_disable_invert_iq()
{
  lora_write_register(REG_INVERTIQ, 0x27);
  lora_write_register(REG_INVERTIQ2, 0x1d);
}

void lora_set_gain(uint8_t gain)
{
  // check allowed range
  if (gain > 6)
  {
    gain = 6;
  }

  // set to standby
  lora_idle();

  // set gain
  if (gain == 0)
  {
    // if gain = 0, enable AGC
    lora_write_register(REG_MODEM_CONFIG_3, 0x04);
  }
  else
  {
    // disable AGC
    lora_write_register(REG_MODEM_CONFIG_3, 0x00);

    // clear Gain and set LNA boost
    lora_write_register(REG_LNA, 0x03);

    // set gain
    lora_write_register(REG_LNA, lora_read_register(REG_LNA) | (gain << 5));
  }
}

