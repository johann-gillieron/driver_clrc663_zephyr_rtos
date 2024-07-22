/*
 * Copyright (c) 2024 Johann Gilliéron
 * Based on the work of: 2016 Ivor Wanders
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_clrc663

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "clrc663_defines.h"
#include "clrc663.h"

LOG_MODULE_REGISTER(clrc663, CONFIG_CLRC663_LOG_LEVEL);

struct clrc663_config {
	const struct spi_dt_spec spi;
  struct gpio_dt_spec shutdown;
	struct gpio_dt_spec irq;
};

struct clrc663_data {
  uint8_t version; // The version of the CLRC663
  uint8_t protocol; // The protocol to use (ISO14443A_106, ISO14443A_212, ...)
  uint8_t tx_DR; // Data rate send to PICC (106, 212, 424, 848 kbps) 
  uint8_t rx_DR; // Data rate receive from PICC (106, 212, 424, 848 kbps)
  bool rf_field_on; // RF field is on
  bool ATS_received; // if card is layer 4 compliant
  bool layer4_activate; // if the layer 4 is activated
  bool CID_support; // if the card support CID
  bool NAD_support; // if the card support NAD
  uint8_t PCB_block_number; // Protocol Control Byte block number
  uint8_t CID; // Card Identifier field
  uint8_t transmit_max_frame_size; // maximal frame size for transmission
  uint16_t SFGT; // Start Frame Guard Time (in ms) minimum time before a RX frame after a TX
  uint16_t FWT; // Frame Waiting Time (in ms) maximum time between two frames
  bool DSM_DRM_same; // if the data rate send and receive needs to be the same
  uint8_t DSM; // Data rate send to card
  uint8_t DRM; // Data rate receive from card
  bool LPCD_enabled; // if the Low Power Card Detection is enabled
  uint8_t RxAna_register; // The value of the RxAna register

  /* In dev
	const struct device *dev;
  const uint8_t DrvMod;
  const uint8_t TxAmpli;
  const uint8_t DrvConf;
  const uint8_t TxRegister;
  */
#ifdef CONFIG_CLRC663_TRIGGER
	/** RX queue buffer. */
	uint8_t rx_queue_buf[SPI_MSG_QUEUE_LEN * SPI_MAX_MSG_LEN];
	/** RX queue. */
	struct k_msgq rx_queue;
	/** Trigger work queue. */
	struct k_work trig_work;
	/** Touch GPIO callback. */
	struct gpio_callback irq_cb;
	/** Semaphore for TX. */
	struct k_sem sem;
	/** Self reference (used in work queue context). */
	const struct device *dev;
#endif /* CONFIG_CLRC663_TRIGGER */
};

const uint16_t fsci_to_bytes[] = CLRC663_14443P4_FSCI; // FSCI to byte conversion table

// ---------------------------------------------------------------------------
// External interaction functions. (private functions)
// ---------------------------------------------------------------------------
/*
 * Transfer (send and receive) data over SPI.
 * @param dev: The device configuration.
 * @param tx_data: The data to send.
 * @param rx_data: The buffer to receive the data.
 * @param len: The length of the data to send.
 * @param last_transfer: Whether this is the last transfer in a sequence.
 * 
 * @return 0 if successful, negative error code otherwise.
 */
int clrc663_SPI_transfer(const struct device *dev, uint8_t *tx_data, uint8_t *rx_data, uint8_t len, bool last_transfer) {
	const struct clrc663_config *config = dev->config;
	struct spi_buf tx_buf = {
		.buf = tx_data,
		.len = len
	};
	struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1
	};
	struct spi_buf rx_buf = {
		.buf = rx_data,
		.len = len
	};
	struct spi_buf_set rx_bufs = {
		.buffers = &rx_buf,
		.count = 1
	};
  int ret;
  ret = spi_transceive_dt(&config->spi, &tx_bufs, &rx_bufs);
  if (last_transfer) {
    spi_release_dt(&config->spi);
  }
	return ret;
}

/*
 * Power off the CLRC663.
 * @param dev: The device configuration.
 */
void clrc663_poweroff(const struct device *dev) {
  const struct clrc663_config *config = dev->config;
  gpio_pin_set_dt(&config->shutdown, 0);
  LOG_DBG("CLRC663 PowerOff");
}

/*
 * Power on the CLRC663.
 * @param dev: The device configuration.
 */
void clrc663_poweron(const struct device *dev) {
  const struct clrc663_config *config = dev->config;
  gpio_pin_set_dt(&config->shutdown, 1);
  k_sleep(K_MSEC(40));
  LOG_DBG("CLRC663 PowerOn");
}

// ---------------------------------------------------------------------------
// Register interaction functions. (private functions)
// ---------------------------------------------------------------------------
/*
 * Read a single register.
 * @param dev: The device configuration.
 * @param reg: The register to read from.
 *
 * @return The value of the register.
 */
uint8_t clrc663_read_reg(const struct device *dev, uint8_t reg) {
  uint8_t instruction_tx[2] = {(reg << 1) | 0x01, 0x00};
  uint8_t instruction_rx[2];
  int ret;
  ret = clrc663_SPI_transfer(dev, instruction_tx, instruction_rx, 2, true);
  if (ret) {
    LOG_ERR("SPI transfer failed");
    return 0;
  }
  
  return instruction_rx[1];  // the second byte the returned value.
}

/*
 * Write a single register.
 * @param dev: The device configuration.
 * @param reg: The register to write to.
 * @param value: The value to write to the register.
 */
void clrc663_write_reg(const struct device *dev, uint8_t reg, uint8_t value) {
	uint8_t instruction_tx[] = {(reg << 1) | 0x00 , value};
	uint8_t discard[2] = {0};
  int ret;

  ret = clrc663_SPI_transfer(dev, instruction_tx, discard, 2, true);
  if (ret) {
    LOG_ERR("SPI transfer failed");
  }
}

/*
 * Write multiple registers.
 * @param dev: The device configuration.
 * @param reg: The register to start writing to.
 * @param values: The values to write to the registers.
 * @param len: The number of values to write.
 */
void clrc663_write_regs(const struct device *dev, const uint8_t reg, const uint8_t* values, uint8_t len) {
	uint8_t instruction_tx[len+1];
	uint8_t discard[len+1];
  int ret;

	instruction_tx[0] = (reg << 1) | 0x00;
  bytecpy(instruction_tx+1, values, len);
	
  ret = clrc663_SPI_transfer(dev, instruction_tx, discard, len+1, true);
  if (ret) {
    LOG_ERR("SPI transfer failed");
  }
}

/*
 * Write to the FIFO.
 * @param dev: The device configuration.
 * @param data: The data to write to the FIFO.
 * @param len: The length of the data to write.
 */
void clrc663_write_fifo(const struct device *dev, const uint8_t* data, uint16_t len) {
	uint8_t write_instruction[len+1];
	uint8_t discard[len];

  write_instruction[0] = (CLRC663_REG_FIFODATA << 1) | 0x00;
  bytecpy(write_instruction+1, data, len);
  
  int ret;
	ret = clrc663_SPI_transfer(dev, write_instruction, discard, len+1, true);
  if (ret) {
    LOG_ERR("SPI transfer failed");
  }
}

/*
 * Read from the FIFO.
 * @param dev: The device configuration.
 * @param rx: The buffer to read the data into.
 * @param len: The length of the data to read.
 */
void clrc663_read_fifo(const struct device *dev, uint8_t* rx, uint16_t len) {
  uint8_t read_instruction[len+1];
  uint8_t *data_in = k_calloc(len+1, sizeof(uint8_t));
  if (data_in == NULL) {
    LOG_ERR("read fifo : Memory allocation failed");
    return;
  }

  read_instruction[0] = (CLRC663_REG_FIFODATA << 1) | 0x01;
  for (uint16_t i=1; i < len; i++) read_instruction[i] = read_instruction[0];
  read_instruction[len] = 0;

  int ret;
  ret = clrc663_SPI_transfer(dev, read_instruction, data_in, len + 1, true);
  if (ret) {
    LOG_ERR("read fifo : SPI transfer failed");
  }
  bytecpy(rx, data_in+1, len);
  k_free(data_in);
}

// ---------------------------------------------------------------------------
// Command functions. (private functions)
// ---------------------------------------------------------------------------
/*
 * Reset the CLRC663 by software.
 * @param dev: The device configuration.
 *
 * @return: void
 */
void clrc663_cmd_soft_reset(const struct device *dev) {
  clrc663_write_reg(dev, CLRC663_REG_COMMAND, CLRC663_CMD_SOFTRESET);
}

void clrc663_cmd_read_E2(const struct device *dev, uint16_t address, uint16_t length) {
  uint8_t parameters[3] = {(uint8_t) (address >> 8), (uint8_t) (address & 0xFF), length};
  clrc663_flush_fifo(dev);
  clrc663_write_fifo(dev, parameters, 3);
  clrc663_write_reg(dev, CLRC663_REG_COMMAND, CLRC663_CMD_READE2);
}

/*
 * Load registers value from the EEPROM.
 * @param dev: The device configuration.
 * @param address: The address of the EEPROM to load into the register.
 * @param regaddr: The first register address to load.
 * @param length: The number of registers to load.
 */
void clrc663_cmd_load_reg(const struct device *dev, uint16_t address, uint8_t regaddr, uint16_t length) {
  uint8_t parameters[4] = {(uint8_t) (address >> 8), (uint8_t) (address & 0xFF), regaddr, length};
  clrc663_flush_fifo(dev);
  clrc663_write_fifo(dev, parameters, 4);
  clrc663_write_reg(dev, CLRC663_REG_COMMAND, CLRC663_CMD_LOADREG);
}

/*
 * Load a protocol.
 * @param dev: The device configuration.
 * @param rx: The receive protocol.
 * @param tx: The transmit protocol.
 */
void clrc663_cmd_load_protocol(const struct device *dev, uint8_t rx, uint8_t tx) {
  uint8_t parameters[2] = {rx, tx};
  clrc663_flush_fifo(dev);
  clrc663_write_fifo(dev, parameters, 2);
  clrc663_write_reg(dev, CLRC663_REG_COMMAND, CLRC663_CMD_LOADPROTOCOL);
}

/*
 * Perform a send command and and copy receive data in FIFO.
 * @param dev: The device configuration.
 * @param data: The data to send.
 * @param len: The length of the data to send.
 *
 * @return: void
 */
void clrc663_cmd_transceive(const struct device *dev, const uint8_t* data, uint16_t len) {
  clrc663_cmd_idle(dev);
  clrc663_flush_fifo(dev);
  clrc663_write_fifo(dev, data, len);
  clrc663_write_reg(dev, CLRC663_REG_COMMAND, CLRC663_CMD_TRANSCEIVE);
}

/*
 * Break the current command and pass the state machine in idle mode
 * @param dev: The device configuration.
 */
void clrc663_cmd_idle(const struct device *dev) {
  clrc663_write_reg(dev, CLRC663_REG_COMMAND, CLRC663_CMD_IDLE);
}

// ---------------------------------------------------------------------------
// Mifare classic commands functions (private functions)
// ---------------------------------------------------------------------------
void clrc663_cmd_load_key_E2(const struct device *dev, uint8_t key_nr) {
  uint8_t parameters[1] = {key_nr};
  clrc663_flush_fifo(dev);
  clrc663_write_fifo(dev, parameters, 1);
  clrc663_write_reg(dev, CLRC663_REG_COMMAND, CLRC663_CMD_LOADKEYE2);
}

void clrc663_cmd_auth(const struct device *dev, uint8_t key_type, uint8_t block_address, const uint8_t* card_uid) {
  clrc663_cmd_idle(dev);
  uint8_t parameters[6] = {key_type, block_address, card_uid[0], card_uid[1], card_uid[2], card_uid[3]};
  clrc663_flush_fifo(dev);
  clrc663_write_fifo(dev, parameters, 6);
  clrc663_write_reg(dev, CLRC663_REG_COMMAND, CLRC663_CMD_MFAUTHENT);
}

void clrc663_cmd_load_key(const struct device *dev, const uint8_t* key) {
  clrc663_cmd_idle(dev);
  clrc663_flush_fifo(dev);
  clrc663_write_fifo(dev, key, 6);
  clrc663_write_reg(dev, CLRC663_REG_COMMAND, CLRC663_CMD_LOADKEY);
}


// ---------------------------------------------------------------------------
// Utility functions. (private functions)
// ---------------------------------------------------------------------------
/*! \brief Turn on the RF field.
 *
 * This function turns on the RF field.
 *
 * \param[in] dev The device structure.
 *
 * \return 0 on success, negative on error.
*/
static void clrc663_rf_field_on(const struct device *dev) {
  struct clrc663_data *data = dev->data;
  clrc663_write_reg(dev, CLRC663_REG_DRVMOD, 0x8E);
  data->rf_field_on = true;
}

/*! \brief Turn off the RF field.
 *
 * This function turns off the RF field.
 *
 * \param[in] dev The device structure.
 *
 * \return 0 on success, negative on error.
*/
static void clrc663_rf_field_off(const struct device *dev) {
  struct clrc663_data *data = dev->data;
  clrc663_write_reg(dev, CLRC663_REG_DRVMOD, 0x86);
  data->rf_field_on = false;
}

/*
 * Initialize the registers of the CLRC663.
 * @param dev: The device configuration.
 */
void clrc663_init_registers(const struct device *dev) {
  struct clrc663_data *data = dev->data;
  clrc663_cmd_soft_reset(dev);
  k_sleep(K_MSEC(10));
  clrc663_cmd_idle(dev);
  clrc663_cmd_load_reg(dev, 0x00C0, 0x00, 19);
  //clrc663_write_reg(dev, CLRC663_REG_TXAMP, 0x52); // Set the number of bytes in FIFO by default : 0x12
  //clrc663_write_reg(dev, CLRC663_REG_DRVCON, 0x01); // Set the OvershootT2
  //clrc663_write_reg(dev, CLRC663_REG_TXL, 0x06); // Set the OvershootT1
  clrc663_load_predefined_protocol(dev, data->protocol);
  data->LPCD_enabled = false;

  clrc663_rf_field_off(dev); // Switch RF field on and 0x86 to switch it off
}

/*
 * Get the version of the CLRC663.
 * @param dev: The device configuration.
 *
 * @return The version of the CLRC663.
 */
uint8_t clrc663_get_version(const struct device *dev) {
  uint8_t version = clrc663_read_reg(dev, CLRC663_REG_VERSION);
  return version;
}

/*
 * Flush the FIFO.
 * @param dev: The device configuration.
 */
void clrc663_flush_fifo(const struct device *dev) {
  clrc663_write_reg(dev, CLRC663_REG_FIFOCONTROL, 0xB0);
}

/*
 * Get the length of the FIFO.
 * @param dev: The device configuration.
 *
 * @return The length of the FIFO.
 */
uint16_t clrc663_fifo_length(const struct device *dev) {
  // should do 512 byte fifo handling here.
  return clrc663_read_reg(dev, CLRC663_REG_FIFOLENGTH);
}

/*
 * Clear the IRQ0 register.
 * @param dev: The device configuration.
 */
void clrc663_clear_irq0(const struct device *dev) {
  clrc663_write_reg(dev, CLRC663_REG_IRQ0, 0x7F);
}

/*
 * Clear the IRQ1 register.
 * @param dev: The device configuration.
 */
void clrc663_clear_irq1(const struct device *dev) {
  clrc663_write_reg(dev, CLRC663_REG_IRQ1, (uint8_t) ~(1<<7));
}

/*
 * Get the value of the IRQ0 register.
 * @param dev: The device configuration.
 *
 * @return The value of the IRQ0 register.
 */
uint8_t clrc663_irq0(const struct device *dev) {
  return clrc663_read_reg(dev, CLRC663_REG_IRQ0);
}

/*
 * Get the value of the IRQ1 register.
 * @param dev: The device configuration.
 *
 * @return The value of the IRQ1 register.
 */
uint8_t clrc663_irq1(const struct device *dev) {
  return clrc663_read_reg(dev, CLRC663_REG_IRQ1);
}

uint8_t clrc663_transfer_E2_page(const struct device *dev, uint8_t* dest, uint8_t page) {
  clrc663_cmd_read_E2(dev, page*64, 64);
  uint8_t res = clrc663_fifo_length(dev);
  clrc663_read_fifo(dev, dest, 64);
  return res;
}

/*
 * Print the name of the error.
 * @note: this is from a clrc663 datasheet table 32.
 * @param dev: The device configuration.
 * @param error: The error code from the error register.
 * @param text_before_error: The text to display before the error name.
 *
 * @return the number of error and 0 if no error.
 */
uint8_t clrc663_error_name(const struct device *dev, uint8_t error, char* text_before_error) {
  uint8_t nb_error = 0;
  if (error == CLRC663_ERROR_CODE_NO_ERROR) {
    LOG_ERR("%s No error", text_before_error);
  } else {
    if (error & CLRC663_ERROR_CODE_DATA_INTEGRITY) {
      LOG_ERR("%s Data integrity (wrong parity or CRC)", text_before_error);
      nb_error++;
    }
    if (error & CLRC663_ERROR_CODE_PROTOCOL) {
      LOG_ERR("%s Protocol (wrong SOF or EOF for ISO14443)", text_before_error);
      nb_error++;
    }
    if (error & CLRC663_ERROR_CODE_COLLISION_DETECTED) {
      LOG_ERR("%s A collision has occured", text_before_error);
      nb_error++;
    }
    if (error & CLRC663_ERROR_CODE_FIFO_EMPTY) {
      LOG_ERR("%s FIFO is empty (No data to send)", text_before_error);
      nb_error++;
    }
    if (error & CLRC663_ERROR_CODE_FRAME_TOO_SHORT) {
      LOG_ERR("%s Receive frame too short", text_before_error);
      nb_error++;
    }
    if (error & CLRC663_ERROR_CODE_FIFO_OVERFLOW) {
      LOG_ERR("%s FIFO overload (FIFO already full)", text_before_error);
      nb_error++;
    }
    if (error & CLRC663_ERROR_CODE_FIFO_WRITE_CRC) {
      LOG_ERR("%s FIFO write possibly a CRC", text_before_error);
      nb_error++;
    }
    if (error & CLRC663_ERROR_CODE_EEPROM) {
      LOG_ERR("%s EEPROM command, a error has occured", text_before_error);
      nb_error++;
    }
  }
  return nb_error;
}

/*
 * Load a predefined protocol.
 * @param dev: The device configuration.
 * @param protocol: The protocol to load.
 */
void clrc663_load_predefined_protocol(const struct device *dev, uint8_t protocol) {
  uint8_t tx;
  uint8_t rx;
  switch(protocol) {
    case CLRC663_PROTO_ISO14443A_106_MILLER_MANCHESTER:
      tx = CLRC663_PROTO_ISO14443A_106_MILLER_MANCHESTER;
      rx = CLRC663_PROTO_ISO14443A_106_MILLER_MANCHESTER;
      break;
    case CLRC663_PROTO_ISO14443A_212_MILLER_BPSK:
      tx = CLRC663_PROTO_ISO14443A_212_MILLER_BPSK;
      rx = CLRC663_PROTO_ISO14443A_212_MILLER_BPSK;
      break;
    case CLRC663_PROTO_ISO14443A_424_MILLER_BPSK:
      tx = CLRC663_PROTO_ISO14443A_424_MILLER_BPSK;
      rx = CLRC663_PROTO_ISO14443A_424_MILLER_BPSK;
      break;
    case CLRC663_PROTO_ISO14443A_848_MILLER_BPSK:
      tx = CLRC663_PROTO_ISO14443A_848_MILLER_BPSK;
      rx = CLRC663_PROTO_ISO14443A_848_MILLER_BPSK;
      break;
    default:
      tx = CLRC663_PROTO_ISO14443A_106_MILLER_MANCHESTER;
      rx = CLRC663_PROTO_ISO14443A_106_MILLER_MANCHESTER;
      break;
  }
  clrc663_cmd_load_protocol(dev, rx, tx);
}

/*
 * Wait for an IRQ0 or an IRQ1.
 * @param dev: The device configuration.
 * @param irq0_en: The IRQ0 enable byte (status and error IRQ)
 * @param irq1_en: The IRQ1 enable byte (timers and global IRQ)
 *
 * @return: value of the IRQ1 register 8 MSB and IRQ0 register 8 LSB.
 */
uint16_t clrc663_irq_wait(const struct device *dev, uint8_t irq0_en, uint8_t irq1_en) {
  uint8_t irq0_value = 0; // IRQ0 for idle and error IRQ
  uint8_t irq1_value = 0; // IRQ1 for timers and global IRQ
  // clear the IRQs
  clrc663_clear_irq0(dev);
  clrc663_clear_irq1(dev);
  // enable the IRQs
  clrc663_write_reg(dev, CLRC663_REG_IRQ0EN, irq0_en);
  clrc663_write_reg(dev, CLRC663_REG_IRQ1EN, irq1_en);
  // wait for a global IRQ
  irq1_value = clrc663_irq1(dev);
  while(!(irq1_value & CLRC663_IRQ1_GLOBAL_IRQ)) {
    irq1_value = clrc663_irq1(dev);
  }
  irq0_value = clrc663_irq0(dev);
  LOG_DBG("IRQ0 = %x, IRQ1 = %x", irq0_value, irq1_value);
  return (irq1_value << 8) | irq0_value;
}

/*
 * Perform a direct data exchange between the uC and the NFC tag through the CLRC663.
 * @param dev: The device configuration.
 * @param tx: The buffer of data to send.
 * @param tx_len: The length of the data to send.
 * @param rx: The buffer for receive data.
 * @param rx_len: The length max of the data to receive.
 *
 * @return the number of bytes received or -1 if an error occured.
 */
int16_t clrc663_indataexchange(const struct device *dev, uint8_t* tx_buffer, uint16_t tx_data_len, uint8_t* rx_buffer, uint16_t rx_data_len) {
  struct clrc663_data *data = dev->data;
  uint8_t irq1_value = 0;
  uint8_t irq0_value = 0;
  uint8_t timer_for_timeout;
  uint16_t timeout_value = (data->FWT + CLRC663_MAX_RX_TIME) * 212; // in 212kHz clock
  uint16_t in_fifo_len;
  uint16_t irq_status;

  if (!(data->rf_field_on)) {
    LOG_ERR("In data Exchange: RF field is off");
    return CLRC663_COM_CODE_CONFIG_ERROR;
  }

  clrc663_cmd_idle(dev);
  clrc663_flush_fifo(dev);

  // Enable the CRC computation for the data exchange
  clrc663_write_reg(dev, CLRC663_REG_TXCRCPRESET, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_ON);
  clrc663_write_reg(dev, CLRC663_REG_RXCRCCON, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_ON);
  
  // Set the timeout timer
  timer_for_timeout = 0;
  clrc663_set_timout_timer(dev, timer_for_timeout, timeout_value);
  
  // Execute the transceive command
  clrc663_cmd_transceive(dev, tx_buffer, tx_data_len); 

  // Wait for the end of the command and the reception of the data or an error.
  irq_status = clrc663_irq_wait(dev, (CLRC663_IRQ0EN_IDLE_IRQEN | CLRC663_IRQ0EN_ERR_IRQEN), CLRC663_IRQ1EN_TIMER0_IRQEN);
  irq0_value = irq_status & 0xFF;
  irq1_value = (irq_status >> 8) & 0xFF;

  clrc663_cmd_idle(dev);
  
  if (irq0_value & CLRC663_IRQ0_ERR_IRQ) {
    char text_before_err_name[] = "In data Exchange: Error : ";
    uint8_t error = clrc663_read_reg(dev, CLRC663_REG_ERROR);
    clrc663_error_name(dev, error, text_before_err_name);
    return -error + CLRC663_COM_CODE_ERROR_REG_ADD;
  }

  if (irq1_value & (1 << timer_for_timeout)) {
    LOG_DBG("In data Exchange: Timeout");
	  return CLRC663_COM_CODE_TIMEOUT;
  }

  in_fifo_len = clrc663_fifo_length(dev);
  if ((in_fifo_len > 0) && (in_fifo_len <= rx_data_len)) {
    clrc663_read_fifo(dev, rx_buffer, in_fifo_len);
    return in_fifo_len;
  }
  if (in_fifo_len > rx_data_len) {
    LOG_ERR("In data Exchange: Not enough space in buffer : %d data receive", in_fifo_len);
    return CLRC663_COM_CODE_BUFFER_OVERFLOW;
  }

  LOG_DBG("In data Exchange: No data received");
  return CLRC663_COM_CODE_NO_DATA;
}

// ---------------------------------------------------------------------------
// Timer functions (private functions)
// ---------------------------------------------------------------------------
void clrc663_activate_timer(const struct device *dev, uint8_t timer, uint8_t active) {
  clrc663_write_reg(dev, CLRC663_REG_TCONTROL, ((active << timer) << 4) | (1 << timer));
}

void clrc663_timer_set_control(const struct device *dev, uint8_t timer, uint8_t value) {
  clrc663_write_reg(dev, CLRC663_REG_T0CONTROL + (5 * timer), value);
}

void clrc663_timer_set_reload(const struct device *dev, uint8_t timer, uint16_t value) {
  clrc663_write_reg(dev, CLRC663_REG_T0RELOADHI + (5 * timer), value >> 8);
  clrc663_write_reg(dev, CLRC663_REG_T0RELOADLO + (5 * timer), value & 0xFF);
}

void clrc663_timer_set_value(const struct device *dev, uint8_t timer, uint16_t value) {
  clrc663_write_reg(dev, CLRC663_REG_T0COUNTERVALHI + (5 * timer), value >> 8);
  clrc663_write_reg(dev, CLRC663_REG_T0COUNTERVALLO + (5 * timer), value & 0xFF);
}

uint16_t clrc663_timer_get_value(const struct device *dev, uint8_t timer) {
  uint16_t res = clrc663_read_reg(dev, CLRC663_REG_T0COUNTERVALHI + (5 * timer)) << 8;
  res += clrc663_read_reg(dev, CLRC663_REG_T0COUNTERVALLO + (5 * timer));
  return res;
}

/*
 * Set the timeout timer (clock @ 211,875 kHz).
 * @param dev: The device configuration.
 * @param timer: The timer to set.
 * @param value: The value to set the timer : value = timeout_time[s] x 212e3.
 */
void clrc663_set_timout_timer(const struct device *dev, uint8_t timer, uint16_t value) {
  clrc663_timer_set_control(dev, timer, CLRC663_TCONTROL_CLK_211KHZ | CLRC663_TCONTROL_START_TX_END);
  clrc663_timer_set_reload(dev, timer, value);
  clrc663_timer_set_value(dev, timer, value);
}

// ---------------------------------------------------------------------------
// From documentation (private functions)
// ---------------------------------------------------------------------------
void clrc663_AN11145_start_IQ_measurement(const struct device *dev) {
  // Part-1, configurate LPCD Mode
  // Please remove any PICC from the HF of the reader.
  // "I" and the "Q" values read from reg 0x42 and 0x43
  // shall be used in part-2 "Detect PICC"
  //  reset CLRC663 and idle
  clrc663_write_reg(dev, 0, 0x1F);
  k_sleep(K_MSEC(50));
  clrc663_write_reg(dev, 0, 0);
  // disable IRQ0, IRQ1 interrupt sources
  clrc663_write_reg(dev, 0x06, 0x7F);
  clrc663_write_reg(dev, 0x07, 0x7F);
  clrc663_write_reg(dev, 0x08, 0x00);
  clrc663_write_reg(dev, 0x09, 0x00);
  clrc663_write_reg(dev, 0x02, 0xB0);  // Flush FIFO
  // LPCD_config
  clrc663_write_reg(dev, 0x3F, 0xC0);  // Set Qmin register
  clrc663_write_reg(dev, 0x40, 0xFF);  // Set Qmax register
  clrc663_write_reg(dev, 0x41, 0xC0);  // Set Imin register
  clrc663_write_reg(dev, 0x28, 0x89);  // set DrvMode register
  // Execute trimming procedure
  clrc663_write_reg(dev, 0x1F, 0x00);  // Write default. T3 reload value Hi
  clrc663_write_reg(dev, 0x20, 0x10);  // Write default. T3 reload value Lo
  clrc663_write_reg(dev, 0x24, 0x00);  // Write min. T4 reload value Hi
  clrc663_write_reg(dev, 0x25, 0x05);  // Write min. T4 reload value Lo
  clrc663_write_reg(dev, 0x23, 0xF8);  // Config T4 for AutoLPCD&AutoRestart.Set AutoTrimm bit.Start T4.
  clrc663_write_reg(dev, 0x43, 0x40);  // Clear LPCD result
  clrc663_write_reg(dev, 0x38, 0x52);  // Set Rx_ADCmode bit
  clrc663_write_reg(dev, 0x39, 0x03);  // Raise receiver gain to maximum
  clrc663_write_reg(dev, 0x00, 0x01);  // Execute Rc663 command "Auto_T4" (Low power card detection and/or Auto trimming)
}

void clrc663_AN11145_stop_IQ_measurement(const struct device *dev, uint8_t *Ires, uint8_t *Qres) {
  // Flush cmd and Fifo
  clrc663_write_reg(dev, 0x00, 0x00);
  clrc663_write_reg(dev, 0x02, 0xB0);
  clrc663_write_reg(dev, 0x38, 0x12);  // Clear Rx_ADCmode bit
  //> ------------ I and Q Value for LPCD ----------------
  *Ires = clrc663_read_reg(dev, CLRC663_REG_LPCD_I_RESULT); //& 0x3F
  *Qres = clrc663_read_reg(dev, CLRC663_REG_LPCD_Q_RESULT); //& 0x3F
}

void clrc663_AN11145_activate_LPCD(const struct device *dev, uint8_t Imesured, uint8_t Qmesured, uint8_t threshold) {
  struct clrc663_data *data = dev->data;
  if(!(data->LPCD_enabled))
  {
    // Activate LPCD
    uint8_t bQmin = Qmesured - threshold;
    uint8_t bQmax = Qmesured + threshold;
    uint8_t bImin = Imesured - threshold;
    uint8_t bImax = Imesured + threshold;
    // check if the values are in the range
    if (bQmin > bQmax) bQmin = 0;
    if (bImin > bImax) bImin = 0;
    //calculate the values for the registers
    uint8_t and_result = bImax & 0x30;
    uint8_t shift_result = and_result << 2;
    uint8_t set_3F_reg = shift_result | bQmin;
    and_result = bImax & 0x0C;
    shift_result = and_result << 4;
    uint8_t set_40_reg = shift_result | bQmax;
    and_result = bImax & 0x03;
    shift_result = and_result << 6;
    uint8_t set_41_reg = shift_result | bImin;

    clrc663_write_reg(dev, 0x3F, set_3F_reg);  // Set Qmin register
    clrc663_write_reg(dev, 0x40, set_40_reg);  // Set Qmax register
    clrc663_write_reg(dev, 0x41, set_41_reg);  // Set Imin register
    // prepare the LPCD command
    // timing: T3(standby time) = 0x07F2 (2034) (1017ms), T4(RFon time) = 0x0013(19) (9ms)
    clrc663_write_reg(dev, 0x1F, 0x07); // Write T3 reload value Hi
    clrc663_write_reg(dev, 0x20, 0xF2); // Write T3 reload value Lo
    clrc663_write_reg(dev, 0x24, 0x00); // Write T4 reload value Hi
    clrc663_write_reg(dev, 0x25, 0x13); // Write T4 reload value Lo
    // next
    clrc663_write_reg(dev, 0x23, 0xDF); // Configure T4 for AutoLPCD and AutoRestart/Autowakeup. Use 2Khz LFO, Start T4
    clrc663_write_reg(dev, 0x43, 0x40); // Clear LPCD result
    clrc663_write_reg(dev, 0x38, 0x52); // Set Rx_ADCmode bit
    data->RxAna_register = clrc663_read_reg(dev, 0x39); // save the RxAna register
    clrc663_write_reg(dev, 0x39, 0x03); // Raise receiver gain to maximum
    // wait for L_waitT4Started
    while((clrc663_read_reg(dev, 0x23) & 0x80) == 0) {
      k_sleep(K_USEC(200));
    }
    // Flush cmd and FIFO. Clear all IRQ flags 
    clrc663_write_reg(dev, 0x00, 0x00);
    clrc663_write_reg(dev, 0x02, 0xB0);
    clrc663_write_reg(dev, 0x06, 0x7F);
    clrc663_write_reg(dev, 0x07, 0x7F);
    // Enable IRQ sources: Idle and PLCD
    clrc663_write_reg(dev, 0x08, 0x10);
    clrc663_write_reg(dev, 0x09, 0x60);
    // Execute LPCD command
    clrc663_write_reg(dev, 0x00, 0x81);
    data->LPCD_enabled = true;
  }
}

/*! \brief End the LPCD mode.
 *
 * This function ends the LPCD mode.
 *
 * \param[in] dev The device structure.
 *
 * \return 0 on success, negative on error.
*/
int8_t clrc663_AN11145_end_LPCD(const struct device *dev) {
  struct clrc663_data *data = dev->data;
  if(data->LPCD_enabled)
  {
    uint8_t version = clrc663_get_version(dev);
    if(version != data->version) {
      return -1; // LPCD not finished
    }
    // Disable LPCD
    // Flush any running command and FIFO 
    clrc663_write_reg(dev, 0x00, 0x00);
    clrc663_write_reg(dev, 0x08, 0x00);
    clrc663_write_reg(dev, 0x09, 0x00);
    // get irq status
    uint8_t status = clrc663_irq0(dev);
    LOG_WRN("IRQ0 = %x", status);
    clrc663_write_reg(dev, 0x02, 0xB0);
    status = clrc663_read_reg(dev, 0x0A); // Read the error status register
    LOG_WRN("Error status = %x", status);
    // Restore the RxAna register
    clrc663_write_reg(dev, 0x39, data->RxAna_register);
    clrc663_write_reg(dev, 0x38, 0x12); // Clear Rx_ADCmode bit
    clrc663_write_reg(dev, 0x23, 0x5F); // Stop T4
    status = clrc663_read_reg(dev, 0x07); // Read the T4 control register
    LOG_WRN("LPCD-irq1 = %x", status);
    data->LPCD_enabled = false;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// ISO 14443-3A/4 functions (private functions)
// ---------------------------------------------------------------------------
/*
 * Perform a REQA command based on the AN12657.
 * @param dev: The device configuration.
 *
 * @return The len of atqa if successful, 0 if no ATQA, -1 if error IRQ
 */
int16_t clrc663_iso14443a_REQA(const struct device *dev, uint8_t *atqa) {
  struct clrc663_data *data = dev->data;
  uint8_t instr = CLRC663_ISO14443_CMD_REQA;
  uint8_t irq1_value = 0;
  uint8_t irq0_value = 0;
  uint8_t timer_for_timeout;
  uint16_t rx_len;

  clrc663_cmd_idle(dev); // Cancel the previous command and the state machine returns to the Idle state
  if (!(data->rf_field_on)) {
    clrc663_write_reg(dev, CLRC663_REG_DRVMOD, 0x8E); // Switch on the RF field
    data->rf_field_on = true;
  }
  clrc663_write_reg(dev, CLRC663_REG_TXCRCPRESET, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_OFF); // Disable the CRC TX
  clrc663_write_reg(dev, CLRC663_REG_RXCRCCON, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_OFF); // Disable the CRC RX

  // Interrupts configuration
  clrc663_clear_irq0(dev);  // Clear the IRQ0 register
  clrc663_clear_irq1(dev);  // Clear the IRQ1 register
  clrc663_write_reg(dev, CLRC663_REG_IRQ1EN, CLRC663_IRQ1EN_TIMER0_IRQEN);  // only trigger on timer for irq1
  clrc663_write_reg(dev, CLRC663_REG_TXDATANUM, 0x0F);  // Set the number of bits to send
  
  // configure a timeout timer, use timer 0.
  timer_for_timeout = 0;
  clrc663_timer_set_control(dev, timer_for_timeout, CLRC663_TCONTROL_CLK_211KHZ | CLRC663_TCONTROL_START_TX_END);
  clrc663_timer_set_reload(dev, timer_for_timeout, 1000);  // 1000 ticks of 5 usec is 5 ms.
  clrc663_timer_set_value(dev, timer_for_timeout, 1000);
  
  clrc663_cmd_transceive(dev, &instr, 1); // Execute the transceive command

  // block until we are done ~ 5ms
  irq1_value = 0;
  while (!(irq1_value & (1 << timer_for_timeout))) {
    irq1_value = clrc663_irq1(dev);
    // either ERR_IRQ or RX_IRQ or Timer
    if (irq1_value & CLRC663_IRQ1_GLOBAL_IRQ) {
      break;  // stop polling irq1 and quit the timeout loop.
    }
  }

  irq0_value = clrc663_irq0(dev); // Read the IRQ0 register
  if (irq0_value & CLRC663_IRQ0_ERR_IRQ) {
    LOG_ERR("REQA : Error IRQ");
    return -1;
  } else if (!(irq0_value & CLRC663_IRQ0_RX_IRQ)) {
    LOG_DBG("REQA : No RX IRQ");
    return 0;
  }

  rx_len = clrc663_fifo_length(dev);
  if (rx_len == 2) {
    clrc663_read_fifo(dev, atqa, rx_len);
    LOG_HEXDUMP_DBG(atqa, rx_len, "ATQA: ");
    return rx_len;
  }
  LOG_DBG("REQA : No ATQA");
  return 0;
}

/*
 * Perform a WUPA command based on the ISO14443A. (same as REQA but for Wake Up)
 * @param dev: The device configuration.
 *
 * @return The len of atqa if successful, 0 if no ATQA, -1 if error IRQ
 */
int16_t clrc663_iso14443a_WUPA(const struct device *dev, uint8_t *atqa) {
  struct clrc663_data *data = dev->data;
  uint8_t instr = CLRC663_ISO14443_CMD_WUPA;
  uint8_t irq1_value = 0;
  uint8_t irq0_value = 0;
  uint8_t timer_for_timeout;
  uint16_t rx_len;

  clrc663_cmd_idle(dev); // Cancel the previous command and the state machine returns to the Idle state
  if (!(data->rf_field_on)) {
    clrc663_write_reg(dev, CLRC663_REG_DRVMOD, 0x8E); // Switch on the RF field
    data->rf_field_on = true;
  }
  clrc663_write_reg(dev, CLRC663_REG_TXCRCPRESET, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_OFF); // Disable the CRC TX
  clrc663_write_reg(dev, CLRC663_REG_RXCRCCON, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_OFF); // Disable the CRC RX

  // Interrupts configuration
  clrc663_clear_irq0(dev);  // Clear the IRQ0 register
  clrc663_clear_irq1(dev);  // Clear the IRQ1 register
  clrc663_write_reg(dev, CLRC663_REG_IRQ1EN, CLRC663_IRQ1EN_TIMER0_IRQEN);  // only trigger on timer for irq1
  clrc663_write_reg(dev, CLRC663_REG_TXDATANUM, 0x0F);  // Set the number of bits to send
  
  // configure a timeout timer, use timer 0.
  timer_for_timeout = 0;
  clrc663_timer_set_control(dev, timer_for_timeout, CLRC663_TCONTROL_CLK_211KHZ | CLRC663_TCONTROL_START_TX_END);
  clrc663_timer_set_reload(dev, timer_for_timeout, 1000);  // 1000 ticks of 5 usec is 5 ms.
  clrc663_timer_set_value(dev, timer_for_timeout, 1000);
  
  clrc663_cmd_transceive(dev, &instr, 1); // Execute the transceive command

  // block until we are done ~ 5ms
  irq1_value = 0;
  while (!(irq1_value & (1 << timer_for_timeout))) {
    irq1_value = clrc663_irq1(dev);
    // either ERR_IRQ or RX_IRQ or Timer
    if (irq1_value & CLRC663_IRQ1_GLOBAL_IRQ) {
      break;  // stop polling irq1 and quit the timeout loop.
    }
  }

  irq0_value = clrc663_irq0(dev); // Read the IRQ0 register
  if (irq0_value & CLRC663_IRQ0_ERR_IRQ) {
    LOG_ERR("WUPA : Error IRQ");
    return -1;
  } else if (!(irq0_value & CLRC663_IRQ0_RX_IRQ)) {
    LOG_DBG("WUPA : No RX IRQ");
    return 0;
  }

  rx_len = clrc663_fifo_length(dev);
  if (rx_len == 2) {
    clrc663_read_fifo(dev, atqa, rx_len);
    LOG_HEXDUMP_DBG(atqa, rx_len, "ATQA: ");
    return rx_len;
  }
  LOG_DBG("REQA : No ATQA");
  return 0;
}

/*
 * Perform WUPA or REQA command based on the norme ISO14443A.
 * @param dev: The device configuration.
 * @param atqa: The buffer for ATQA of the card.
 *
 * @return The len of atqa if successful, 0 if no ATQA, -1 if error IRQ
 */
int16_t clrc663_iso14443a_WUPA_REQA(const struct device *dev, uint8_t *atqa){
  int16_t ret = clrc663_iso14443a_WUPA(dev, atqa);
  if (ret == 0) {
    ret = clrc663_iso14443a_REQA(dev, atqa);
  }
  return ret;
}

/*
 * Perform a select of a NFC tag, solve the collision and read the UID and the SAK.
 * @param dev: The device configuration.
 * @param uid: The buffer for UID of the card.
 * @param sak: The buffer for SAK of the card.
 *
 * @return The UID length.
 */
int8_t clrc663_iso14443a_select(const struct device *dev, uint8_t* uid, uint8_t* sak) {
  struct clrc663_data *data = dev->data;
  clrc663_cmd_idle(dev);
  if (!(data->rf_field_on)) {
    clrc663_load_predefined_protocol(dev, data->protocol); // Load the protocol
    clrc663_write_reg(dev, CLRC663_REG_DRVMOD, 0x8E); // Switch on the RF field
    data->rf_field_on = true;
  }
  clrc663_flush_fifo(dev);

  // enable the global IRQ for Rx done and Errors.
  clrc663_write_reg(dev, CLRC663_REG_IRQ0EN, CLRC663_IRQ0EN_RX_IRQEN | CLRC663_IRQ0EN_ERR_IRQEN);
  clrc663_write_reg(dev, CLRC663_REG_IRQ1EN, CLRC663_IRQ1EN_TIMER0_IRQEN);  // only trigger on timer for irq1
  clrc663_write_reg(dev, CLRC663_REG_TXDATANUM, 0x00);  // Set the number of bits to send

  // configure a timeout timer, use timer 0.
  uint8_t timer_for_timeout = 0;

  // Set timer to 221 kHz clock, start at the end of Tx.
  clrc663_timer_set_control(dev, timer_for_timeout, CLRC663_TCONTROL_CLK_211KHZ | CLRC663_TCONTROL_START_TX_END);
  // Frame waiting time: FWT = (256 x 16/fc) x 2 FWI
  // FWI defaults to four... so that would mean wait for a maximum of ~ 5ms

  clrc663_timer_set_reload(dev, timer_for_timeout, 1000);  // 1000 ticks of 5 usec is 5 ms.
  clrc663_timer_set_value(dev, timer_for_timeout, 1000);
  uint8_t cascade_level;
  for (cascade_level=1; cascade_level <= 3; cascade_level++) {
    uint8_t cmd = 0;
    uint8_t known_bits = 0;  // known bits of the UID at this level so far.
    uint8_t send_req[7] = {0};  // used as Tx buffer.
    uint8_t* uid_this_level = &(send_req[2]);
    // pointer to the UID so far, by placing this pointer in the send_req
    // array we prevent copying the UID continuously.
    uint8_t message_length;

    switch (cascade_level) {
      case 1:
        cmd = CLRC663_ISO14443_CAS_LEVEL_1;
        break;
      case 2:
        cmd = CLRC663_ISO14443_CAS_LEVEL_2;
        break;
      case 3:
        cmd = CLRC663_ISO14443_CAS_LEVEL_3;
        break;
    }

    // disable CRC in anticipation of the anti collision protocol
    clrc663_write_reg(dev, CLRC663_REG_TXCRCPRESET, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_OFF);
    clrc663_write_reg(dev, CLRC663_REG_RXCRCCON, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_OFF);


    // max 32 loops of the collision loop.
    uint8_t collision_n;
    for (collision_n=0; collision_n < 32; collision_n++) {
      // clear interrupts
      clrc663_clear_irq0(dev);
      clrc663_clear_irq1(dev);

      send_req[0] = cmd;
      send_req[1] = 0x20 + known_bits;
      // send_req[2..] are filled with the UID via the uid_this_level pointer.

      // Only transmit the last 'x' bits of the current byte we are discovering
      // First limit the txdatanum, such that it limits the correct number of bits.
      clrc663_write_reg(dev, CLRC663_REG_TXDATANUM, (known_bits % 8) | CLRC663_TXDATANUM_DATAEN);

      // ValuesAfterColl: If cleared, every received bit after a collision is
      // replaced by a zero. This function is needed for ISO/IEC14443 anticollision (0<<7).
      // We want to shift the bits with RxAlign
      uint8_t rxalign = known_bits % 8;
      LOG_DBG("Setting rx align to: %hhd\n", rxalign);


      // then sent the send_req to the hardware,
      // (known_bits / 8) + 1): The ceiled number of bytes by known bits.
      // +2 for cmd and NVB.
      if ((known_bits % 8) == 0) {
        message_length = ((known_bits / 8)) + 2;
      } else {
        message_length = ((known_bits / 8) + 1) + 2;
      }

      LOG_DBG("Send:%hhd long: ", message_length);

      clrc663_cmd_transceive(dev, send_req, message_length);


      // block until we are done
      uint8_t irq1_value = 0;
      while (!(irq1_value & (1 << timer_for_timeout))) {
        irq1_value = clrc663_irq1(dev);
        // either ERR_IRQ or RX_IRQ or Timer
        if (irq1_value & CLRC663_IRQ1_GLOBAL_IRQ) {
          break;  // stop polling irq1 and quit the timeout loop.
        }
      }
      clrc663_cmd_idle(dev);


      // next up, we have to check what happened.
      uint8_t irq0 = clrc663_irq0(dev);
      uint8_t error = clrc663_read_reg(dev, CLRC663_REG_ERROR);
      uint8_t coll = clrc663_read_reg(dev, CLRC663_REG_RXCOLL);
      LOG_DBG("irq0: %hhX\n", irq0);
      LOG_DBG("error: %hhX\n", error);
      uint8_t collision_pos = 0;
      if (irq0 & CLRC663_IRQ0_ERR_IRQ) {  // some error occured.
        // Check what kind of error.
        // error = clrc663_read_reg(CLRC663_REG_ERROR);
        if (error & CLRC663_ERROR_COLLDET) {
          // A collision was detected...
          if (coll & (1<<7)) {
            collision_pos = coll & (~(1<<7));
            LOG_DBG("Collision at %hhX\n", collision_pos);
            // This be a true collision... we have to select either the address
            // with 1 at this position or with zero
            // ISO spec says typically a 1 is added, that would mean:
            // uint8_t selection = 1;

            // However, it makes sense to allow some kind of user input for this, so we use the
            // current value of uid at this position, first index right byte, then shift such
            // that it is in the rightmost position, ten select the last bit only.
            // We cannot compensate for the addition of the cascade tag, so this really
            // only works for the first cascade level, since we only know whether we had
            // a cascade level at the end when the SAK was received.
            uint8_t choice_pos = known_bits + collision_pos;
            uint8_t selection = (uid[((choice_pos + (cascade_level-1)*3)/8)] >> ((choice_pos) % 8))&1;


            // We just OR this into the UID at the right position, later we
            // OR the UID up to this point into uid_this_level.
            uid_this_level[((choice_pos)/8)] |= selection << ((choice_pos) % 8);
            known_bits++;  // add the bit we just decided.

            LOG_DBG("uid_this_level now kb %hhd long: ", known_bits);

          } else {
            // Datasheet of clrc663:
            // bit 7 (CollPosValid) not set:
            // Otherwise no collision is detected or
            // the position of the collision is out of the range of bits CollPos.
            LOG_DBG("Collision but no valid collpos.\n");
            collision_pos = 0x20 - known_bits;
          }
        } else {
          // Can this ever occur?
          collision_pos = 0x20 - known_bits;
          LOG_DBG("No collision, error was: %hhx, setting collision_pos to: %hhx\n", error, collision_pos);
        }
      } else if (irq0 & CLRC663_IRQ0_RX_IRQ) {
        // we got data, and no collisions, that means all is well.
        collision_pos = 0x20 - known_bits;
        LOG_DBG("Got data, no collision, setting to: %hhx\n", collision_pos);
      } else {
        // We have no error, nor received an RX. No response, no card?
        return 0;
      }
      LOG_DBG("collision_pos: %hhX\n", collision_pos);

      // read the UID Cln so far from the buffer.
      uint8_t rx_len = clrc663_fifo_length(dev);
      uint8_t buf[5];  // Size is maximum of 5 bytes, UID[0-3] and BCC.

      clrc663_read_fifo(dev, buf, rx_len < 5 ? rx_len : 5);

      LOG_DBG("Fifo %hhd long: ", rx_len);

      LOG_DBG("uid_this_level kb %hhd long: ", known_bits);
      // move the buffer into the uid at this level, but OR the result such that
      // we do not lose the bit we just set if we have a collision.
      uint8_t rbx;
      for (rbx = 0; (rbx < rx_len); rbx++) {
        uid_this_level[(known_bits / 8) + rbx] |= buf[rbx];
      }
      known_bits += collision_pos;
      LOG_DBG("known_bits: %hhX\n", known_bits);

      if ((known_bits >= 32)) {
        LOG_DBG("exit collision loop: uid_this_level kb %hhd long: ", known_bits);

        break;  // done with collision loop
      }
    }  // end collission loop

    // check if the BCC matches
    uint8_t bcc_val = uid_this_level[4];  // always at position 4, either with CT UID[0-2] or UID[0-3] in front.
    uint8_t bcc_calc = uid_this_level[0]^uid_this_level[1]^uid_this_level[2]^uid_this_level[3];
    if (bcc_val != bcc_calc) {
      LOG_ERR("Something went wrong, BCC does not match.\n");
      return 0;
    }

    // clear interrupts
    clrc663_clear_irq0(dev);
    clrc663_clear_irq1(dev);

    send_req[0] = cmd;
    send_req[1] = 0x70;
    // send_req[2,3,4,5] // contain the CT, UID[0-2] or UID[0-3]
    send_req[6] = bcc_calc;
    message_length = 7;

    // Ok, almost done now, we reenable the CRC's
    clrc663_write_reg(dev, CLRC663_REG_TXCRCPRESET, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_ON);
    clrc663_write_reg(dev, CLRC663_REG_RXCRCCON, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_ON);

    // reset the Tx and Rx registers (disable alignment, transmit full bytes)
    clrc663_write_reg(dev, CLRC663_REG_TXDATANUM, (known_bits % 8) | CLRC663_TXDATANUM_DATAEN);
    uint8_t rxalign = 0;
    clrc663_write_reg(dev, CLRC663_REG_RXBITCTRL, (0 << 7) | (rxalign << 4));

    // actually send it!
    clrc663_cmd_transceive(dev, send_req, message_length);

    // Block until we are done...
    uint8_t irq1_value = 0;
    while (!(irq1_value & (1 << timer_for_timeout))) {
      irq1_value = clrc663_irq1(dev);
      if (irq1_value & CLRC663_IRQ1_GLOBAL_IRQ) {  // either ERR_IRQ or RX_IRQ
        break;  // stop polling irq1 and quit the timeout loop.
      }
    }
    clrc663_cmd_idle(dev);

    // Check the source of exiting the loop.
    uint8_t irq0_value = clrc663_irq0(dev);
    if (irq0_value & CLRC663_IRQ0_ERR_IRQ) {
      // Check what kind of error.
      uint8_t error = clrc663_read_reg(dev, CLRC663_REG_ERROR);
      if (error & CLRC663_ERROR_COLLDET) {
        // a collision was detected with NVB=0x70, should never happen.
        LOG_ERR("Collision detected with NVB=0x70, should never happen.");
        return 0;
      }
    }

    // Read the sak answer from the fifo.
    uint8_t sak_len = clrc663_fifo_length(dev);
    if (sak_len != 1) {
      return 0;
    }
    uint8_t sak_value;
    clrc663_read_fifo(dev, &sak_value, sak_len);

    if (sak_value & (1 << 2)) {
      // UID not yet complete, continue with next cascade.
      // This also means the 0'th byte of the UID in this level was CT, so we
      // have to shift all bytes when moving to uid from uid_this_level.
      uint8_t UIDn;
      for (UIDn=0; UIDn < 3; UIDn++) {
        // uid_this_level[UIDn] = uid_this_level[UIDn + 1];
        uid[(cascade_level-1)*3 + UIDn] = uid_this_level[UIDn + 1];
      }
    } else {
      // Done according so SAK!
      // Add the bytes at this level to the UID.
      uint8_t UIDn;
      for (UIDn=0; UIDn < 4; UIDn++) {
        uid[(cascade_level-1)*3 + UIDn] = uid_this_level[UIDn];
      }

      *sak = sak_value;
      // Finally, return the length of the UID that's now at the uid pointer.
      return cascade_level*3 + 1;
    }
  }  // cascade loop
  return 0;  // getting an UID failed.
}

/*
 * Perform a RATS command based on the AN12657 for the layer 14443-4.
 * @param dev: The device configuration.
 * @param ats: The buffer for the ATS of the card [buffer of 64 uint8]
 *
 * @return 0 if no ATS was received, -1 if an error occured or the length of the ATS if success.
 */
int16_t clrc663_iso14443p4_RATS(const struct device *dev, uint8_t *ats) {
  struct clrc663_data *data = dev->data;
  uint8_t send_req[2];
  send_req[0] = CLRC663_14443P4_RATS_START_BYTE;
  send_req[1] = CLRC663_14443P4_RATS_256 | (data->CID & CLRC663_14443P4_RATS_CID_MASK); // FSD = 256 bytes (maximal size of the frame from the PICC to the PCD)
  uint16_t req_length = sizeof(send_req);
  uint16_t rx_length = 256; // The maximal size of the ATS define by the RATS command
  uint8_t *rx_buffer = k_calloc(rx_length, sizeof(uint8_t));
  if (rx_buffer == NULL) {
    LOG_ERR("RATS: Memory allocation failed");
    return -1;
  }

  if (!data->rf_field_on) {
    LOG_ERR("RATS: RF field is off");
    return -1;
  }
  
  data->ATS_received = false;
  data->layer4_activate = false;
  data->SFGT = CLRC663_14443P4_SFGT_DEFAULT; // Default value for the SFGT
  data->FWT = CLRC663_14443P4_FWT_DEFAULT; // Default value for the FWT
  
  int16_t ret = clrc663_indataexchange(dev, send_req, req_length, rx_buffer, rx_length);
  if (ret <= 0) {
    k_free(rx_buffer);
    LOG_DBG("No response from the PICC.");
    return ret;
  } else if (ret > rx_length) {
    LOG_ERR("RATS: ATS too long.");
    k_free(rx_buffer);
    return -1;
  }
  
  bytecpy(ats, rx_buffer, ret);
  data->ATS_received = true;

  LOG_HEXDUMP_DBG(rx_buffer, ret, "ATS: ");
  uint8_t T0 = ats[CLRC663_14443P4_ATS_T0_BYTE];

  // Analyze the ATS based on the ISO 14443-4
  data->transmit_max_frame_size = fsci_to_bytes[T0 & CLRC663_14443P4_ATS_T0_FSCI_MASK];
  if (T0 & CLRC663_14443P4_ATS_T0_TA_PRESENT_MASK) {
    uint8_t TA1 = ats[CLRC663_14443P4_ATS_TA_BYTE];
    data->DSM_DRM_same = ((TA1 & CLRC663_14443P4_ATS_TA_DS_DR_SAME) > 0);
    data->DSM = (TA1 >> CLRC663_14443P4_ATS_TA_DSM_SIFT) & CLRC663_14443P4_ATS_TA_DSM_MASK;
    data->DRM = TA1 & CLRC663_14443P4_ATS_TA_DRM_MASK;
    LOG_DBG("RATS: DSM: %hhX, DRM: %hhX", data->DSM, data->DRM);
  } 
  if (T0 & CLRC663_14443P4_ATS_T0_TB_PRESENT_MASK) {
    uint8_t TB1 = ats[CLRC663_14443P4_ATS_TB_BYTE];
    uint8_t SFGI = TB1 & CLRC663_14443P4_ATS_TB_SFGI_MASK;
    data->SFGT = (uint8_t)((1<<SFGI) * 0.302065 + 0.9);  // Start-up frame guard time in ms SFGT = (256 x 16 x 1000 / fc) x 2^SFGI [ms] (+ 0.9 for rounding up)
    uint8_t FWI = (TB1 >> CLRC663_14443P4_ATS_TB_FWI_SHIFT) & CLRC663_14443P4_ATS_TB_FWI_MASK;
    data->FWT = (uint8_t)((1<<FWI) * 0.302065 + 0.9);  // Frame waiting time in ms FWT = (256 x 16 x 1000 / fc) x 2^FWI [ms]
    LOG_DBG("RATS: SFGI: %hhX, SFGT: %d ms, FWI: %d, FWT: %d ms", SFGI, data->SFGT, FWI, data->FWT);
  }
  if (T0 & CLRC663_14443P4_ATS_T0_TB_PRESENT_MASK) {
    uint8_t TC1 = ats[CLRC663_14443P4_ATS_TC_BYTE];
    data->CID_support = (TC1 >> CLRC663_14443P4_ATS_TC_CID_SUP_SHIFT) & CLRC663_14443P4_ATS_TC_CID_SUP_MASK;
    data->NAD_support = TC1 & CLRC663_14443P4_ATS_TC_NAD_SUP_MASK;
    LOG_DBG("RATS: CID supported: %d, NAD supported: %d,  same data rate ds: %d", data->CID_support, data->NAD_support, data->DSM_DRM_same);
  }

  k_free(rx_buffer);
  return ret;
}

/*
 * Perform a PPS command based on the ISO/IEC 14443-4 for the layer 4.
 * @param dev: The device configuration.
 *
 * @return 0 on success, negative on error.
 */
int16_t clrc663_iso14443p4_PPS(const struct device *dev) {
  struct clrc663_data *data = dev->data;
  data->layer4_activate = false;
  if (!data->ATS_received) {
    LOG_ERR("PPS: No ATS received yet.");
    return CLRC663_COM_CODE_CONFIG_ERROR;
  }

  uint8_t ppss = CLRC663_14443P4_PPS_PPSS_BASE | (data->CID & CLRC663_14443P4_PPS_PPSS_CID_MASK);
  uint8_t pps0 = CLRC663_14443P4_PPS_PPS0_WITHOUT_PPS1;
  uint8_t pps1 = CLRC663_14443P4_PPS_PPS1_106_RX | CLRC663_14443P4_PPS_PPS1_106_TX;
  uint8_t request[] = {ppss, pps0, pps1};
  uint8_t *response = k_calloc(3, sizeof(uint8_t));
  uint8_t request_size = 2;
  int16_t ret = 0;
  LOG_DBG("PPS: Sending PPS command %hhX %hhX %hhX", ppss, pps0, pps1);

  if (response == NULL) {
    LOG_ERR("PPS: Memory allocation failed");
    return -1;
  }
  // Check if the pps1 is needed.
  if (pps0 != CLRC663_14443P4_PPS_PPS0_WITHOUT_PPS1){
    request_size += 1;
  }

  ret = clrc663_indataexchange(dev, request, request_size, response, request_size);
  if (ret <= 0) {
    k_free(response);
    LOG_ERR("PPS: No response");
    return ret;
  }
  LOG_DBG("PPS: Received PPS response %hhX %hhX %hhX", response[0], response[1], response[2]);
  if (response[0] != ppss) {
    k_free(response);
    LOG_INF("PPS: Invalid response");
    return -1;
  }
  LOG_DBG("PPS: Success");
  data->layer4_activate = true;
  data->PCB_block_number = CLRC663_14443P4_BLOCK_NUMBER_DEFAULT; // block number for the next frame i,r,s-BLOCK from ISO 14443-4
  k_free(response);
  return 0;
}

/*
 * Perform a DESELECT command based on the ISO/IEC 14443-4 for the layer 4.
 * @param dev: The device configuration.
 *
 *  @return 0 on success, negative on error.
 */
uint8_t clrc663_iso14443p4_DESELECT(const struct device *dev) {
  struct clrc663_data *data = dev->data;
  uint8_t instr = CLRC663_14443P4_PCB_S_BLOCK_BASE | CLRC663_14443P4_PCB_S_BLOCK_DESELECT;
  uint8_t response;
  int16_t ret;
  data->SFGT = CLRC663_14443P4_SFGT_DEFAULT;
  data->FWT = CLRC663_14443P4_FWT_DEFAULT;

  ret = clrc663_indataexchange(dev, &instr, 1, &response, 1);

  if (ret <= 0) {
    LOG_DBG("DESELECT p4: No response");
    return ret;
  }
  LOG_DBG("DESELECT p4: Received s-block response %hhX", response);
  if (response != instr) {
    LOG_DBG("DESELECT p4: Invalid response");
    return -1;
  }
  data->layer4_activate = false;
  data->ATS_received = false;
  data->PCB_block_number = CLRC663_14443P4_BLOCK_NUMBER_DEFAULT; // PCB for the next frame i-BLOCK from ISO 14443-4
  return 0;
}

/*
 * Perform a HLTA command based on the AN12657 for the layer 14443-3a.
 * @param dev: The device configuration.
 *
 * @return 0 on success, negative on error.
 */
int8_t clrc663_iso14443a_HLTA(const struct device *dev) {
  uint8_t instr[] = {0x50, 0x00};
  clrc663_cmd_idle(dev);
  int16_t ret = clrc663_indataexchange(dev, instr, 2, NULL, 0);
  if (ret < 0) {
    return ret;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// API functions
// ---------------------------------------------------------------------------
/*
 * Initialize the CLRC663 device.
 * @param dev: The device configuration.
 *
 * @return 0 on success, negative on error.
 */
static int clrc663_init(const struct device *dev) {
	const struct clrc663_config *config = dev->config;
  struct clrc663_data *data = dev->data;
	int ret;
	LOG_DBG("Initializing clrc663");
  
	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI not ready");
    return -ENODEV;
	}

	if (!spi_cs_is_gpio_dt(&config->spi)) {
		LOG_ERR("No CS GPIO found");
    return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->spi.config.cs.gpio, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure CS GPIO (%d)", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&config->spi.config.cs.gpio, GPIO_OUTPUT);
  if (ret < 0) {
		LOG_ERR("Could not configure CS GPIO (%d)", ret);
		return ret;
	}

	if (!gpio_is_ready_dt(&config->shutdown)) {
    LOG_ERR("Shutdown GPIO not ready");
		return -EBUSY;
	}

	ret = gpio_pin_configure_dt(&config->shutdown, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure shutdown GPIO (%d)", ret);
		return ret;
	}

	ret = gpio_pin_set_dt(&config->shutdown, HIGH);
	if (ret < 0) {
		LOG_ERR("Could not toggle shutdown(%d)", ret);
		return ret;
	}

  // Power ON the device.
  clrc663_poweron(dev);
  clrc663_init_registers(dev);
  
  // test the SPI connection
  uint8_t version = clrc663_get_version(dev);
  uint8_t version2 = clrc663_get_version(dev);
  if (version == 0 || version == 0xFF) {
    LOG_ERR("CLRC663 not found");
    return -ENODEV;
  }
  if (version != version2) {
    LOG_ERR("CLRC663 not respnoding correctly");
    return -ENODEV;
  }
  data->version = version;

  LOG_DBG("CLRC663 version %hhX initialized", version);
  return 0;
}

/*
 * Detect a card in the field and read the UID and SAK.
 * @param dev: The device configuration.
 * @param uid: Return the UID of the card [buffer of 10 uint8].
 * @param sak: Return the SAK of the card [buffer of 1 uint8].
 * @param atqa: Return the ATQA of the card [buffer of 2 uint8].
 * @param ats: Return the ATS of the card [buffer of 32 uint8].
 *
 * @return uid length on success, 0 on failure.
 */
static uint8_t clrc663_card_select(const struct device *dev, uint8_t uid[10], uint8_t sak[1], uint8_t atqa[2], uint8_t ats[32]) {
  struct clrc663_data *data = dev->data;
  uint8_t uid_len;
  int8_t status;
  int16_t ret;

  // Load the protocol
  if(data->LPCD_enabled){
    status = clrc663_AN11145_end_LPCD(dev);
    if (status < 0) {
      LOG_ERR("LPCD: Error stopping the LPCD");
      return 0;
    }
  }

  if (data->rf_field_on){
    if(data->layer4_activate || data->ATS_received){
      memset(ats, 0, 64); // Reset the ATS buffer
      clrc663_iso14443p4_DESELECT(dev);
      data->ATS_received = false;
      data->layer4_activate = false;
    }
    clrc663_iso14443a_HLTA(dev);
  }
  if (data->protocol != CLRC663_PROTO_ISO14443A_106_MILLER_MANCHESTER) {
    data->protocol = CLRC663_PROTO_ISO14443A_106_MILLER_MANCHESTER;
    clrc663_init_registers(dev);
  }
  if(!(data->rf_field_on)) {
    clrc663_rf_field_on(dev);
    k_sleep(K_MSEC(5)); // Wait for the RF field to be stable
  }

  ret = clrc663_iso14443a_WUPA_REQA(dev, atqa);

  if (ret > 0) {
    // Select the card and discover its uid.
    uid_len = clrc663_iso14443a_select(dev, uid, sak); // Perform select and collision resolution
    if (uid_len > 0) {
      // Check if the card is ISO14443-4 compliant.
      if (sak[0] & 0x20) {
        ret = clrc663_iso14443p4_RATS(dev, ats); // Perform RATS command layer 4 activation
        if (ret > 0) ret = clrc663_iso14443p4_PPS(dev); // Perform PPS command
      }
      return uid_len;
    }
  }
  LOG_DBG("No card detected");
  return 0;
}

// TODO : Handle the S-Block WTX (Wait Time Extension) and R-Block (NAck) in the communication.
/*
 * Perform direct communication by iso 14443-4 with a PICC.
 * @param dev: The device configuration.
 * @param data_tx: The data to send to the tag.
 * @param data_tx_len: The length of the data to send.
 * @param data_rx: The data received from the tag.
 * @param data_rx_len: The length of the data received.
 * 
 * @return received data length on success, negative on error.
 * 
 * @note:
 * TODO : Handle the S-Block WTX (Wait Time Extension) and R-Block (NAck) in the communication.
 * ================================
 * Frame Format for ISO/IEC 14443-4
 * ================================
 *
 * The frame format ISO 14443-4 specifications for block formats.
 * This is the format used by the example firmware, and seen in Figure 3.
 *  - PCB – Protocol Control Byte, this byte is used to transfer format information about each PDU block.
 *  - CID – Card Identifier field, this byte is used to identify specific tags. It contains a 4 bit CID value as well
 *          as information on the signal strength between the reader and the tag.
 *  - NAD – Node Address field, the example firmware does not support the use of NAD.
 *  - INF – Information field
 *  - EDC – CRC of the transmitted block, which is the CRC defined in ISO/IEC 14443-3
 *
 *  |-----|-----|-----|----------------|-----|
 *  | PCB | CID | NAD |      INF       | EDC |
 *  |-----|-----|-----|----------------|-----|
 */
static int16_t clrc663_iso14443_4_communication(const struct device *dev, uint8_t* data_tx, uint16_t data_tx_len, uint8_t* data_rx, uint16_t data_rx_len) {
  struct clrc663_data *data = dev->data;
  uint8_t *rx_buffer;
  uint8_t *tx_buffer;
  uint8_t skip = 1; // skip the first byte of the data received (PCB Byte)
  uint8_t add = 1; // add a first byte to the data to send (PCB Byte)
  data->PCB_block_number = data->PCB_block_number & CLRC663_14443P4_PCB_BLOCK_NUMBER_MASK;
  uint8_t PCB = CLRC663_14443P4_PCB_I_BLOCK_BASE | data->PCB_block_number;
  uint8_t tries = 6; // Number of tries for the communication
  int16_t status = 0;

  if (!(data->layer4_activate)) {
    LOG_ERR("Direct com : Layer 4 not activated");
    return CLRC663_COM_CODE_CONFIG_ERROR;
  }

  if (data_tx_len >= data->transmit_max_frame_size) {
    LOG_ERR("Direct com : Data too long for the communication with the PICC");
    return CLRC663_COM_CODE_TOO_MUCH_DATA_SEND;
  }
  LOG_HEXDUMP_DBG(data_tx, data_tx_len, "Direct com : Data send to function: ");

  // Allocate and initialise the buffers
  rx_buffer = k_calloc(data_rx_len + skip, sizeof(uint8_t));
  tx_buffer = k_calloc(data->transmit_max_frame_size, sizeof(uint8_t));

  if (tx_buffer == NULL) {
    LOG_ERR("Direct com : Memory allocation failed");
    return CLRC663_COM_CODE_CONFIG_ERROR;
  }
  if (rx_buffer == NULL) {
    LOG_ERR("Direct com : Memory allocation failed");
    return CLRC663_COM_CODE_CONFIG_ERROR;
  }

  tx_buffer[0] = PCB;
  bytecpy((tx_buffer + add), data_tx, data_tx_len);
  LOG_HEXDUMP_DBG(data_tx, data_tx_len, "Direct com : Data frame receive by fonction: ");
  LOG_HEXDUMP_DBG(tx_buffer, (data_tx_len + add), "Direct com : Data frame to send: ");

  // Send the data to the PICC
  while(tries > 0) {
    tries -= 1;

    if (status < 0) {
      uint8_t RNAck_PCB = CLRC663_14443P4_PCB_R_BLOCK_ID | data->PCB_block_number;
      status = clrc663_indataexchange(dev, &RNAck_PCB, 1, rx_buffer, (data_rx_len + skip));
    } else {
      status = clrc663_indataexchange(dev, tx_buffer, (data_tx_len + add), rx_buffer, (data_rx_len + skip));
    }

    if (status < 0) {
      if (status == CLRC663_COM_CODE_TIMEOUT) {
        continue; // Retry the communication rule 4 of the ISO 14443-4
      }
      LOG_INF("Direct com : Error during the communication with the PICC");
      break;
    }

    if (status == 0) {
      LOG_INF("Direct com : No data received from the PICC");
      break;
    }

    LOG_HEXDUMP_DBG(rx_buffer, status, "Direct com : RAW Data receive: ");

    // I-Block handling //
    if ((rx_buffer[0] & CLRC663_14443P4_PCB_BLOCK_MASK) == CLRC663_14443P4_PCB_I_BLOCK_ID) {
      // Check the block number
      if ((rx_buffer[0] & CLRC663_14443P4_PCB_BLOCK_NUMBER_MASK) != data->PCB_block_number) {
        LOG_ERR("Direct com : I-BLOCK block number mismatch");
        status = CLRC663_COM_CODE_DATA_FORMAT_ERROR;
        break;
      } else {
        // Toggle the block number for the next frame
        data->PCB_block_number ^= CLRC663_14443P4_PCB_BLOCK_NUMBER_MASK; 
        skip = 1;
      }

      // Check the CID
      if(rx_buffer[0] & CLRC663_14443P4_PCB_CID_MASK) {
        uint8_t CID = rx_buffer[skip] & CLRC663_14443P4_CID_CID_MASK;
        uint8_t power_level = rx_buffer[skip] & CLRC663_14443P4_CID_PWR_LVL_MASK;
        skip += 1;
        LOG_DBG("Direct com : CID not supported at the moment but received : CID = %hhX, power lvl = %hhX", CID, power_level);
      }
      // Check the NAD
      if (rx_buffer[0] & CLRC663_14443P4_PCB_I_BLOCK_NAD_MASK) {
        uint8_t NAD = rx_buffer[skip];
        skip += 1;
        LOG_DBG("Direct com : NAD not supported at the moment but received : NAD = %hhX", NAD);
      }

      break; // Data received correctly
    }
    // R-Block handling //
    else if ((rx_buffer[0] & CLRC663_14443P4_PCB_BLOCK_MASK) == CLRC663_14443P4_PCB_R_BLOCK_ID) {
      // Check the block number
      if ((rx_buffer[0] & CLRC663_14443P4_PCB_BLOCK_NUMBER_MASK) != data->PCB_block_number) {
        LOG_ERR("Direct com : R-BLOCK block number mismatch, resend the last data");
        status = 0;
        continue; // Retry the communication rule 6 of the ISO 14443-4
      } else {
        // Toggle the block number for the next frame
        data->PCB_block_number ^= CLRC663_14443P4_PCB_BLOCK_NUMBER_MASK;
        skip = 1;
      }
      // Check the CID
      if(rx_buffer[0] & CLRC663_14443P4_PCB_CID_MASK) {
        uint8_t CID = rx_buffer[skip] & CLRC663_14443P4_CID_CID_MASK;
        uint8_t power_level = rx_buffer[skip] & CLRC663_14443P4_CID_PWR_LVL_MASK;
        skip += 1;
        LOG_DBG("Direct com : CID not supported at the moment but received : CID = %hhX, power lvl = %hhX", CID, power_level);
      }
      // check aknowledge
      if (rx_buffer[0] & CLRC663_14443P4_PCB_R_BLOCK_ACK_MASK) {
        LOG_DBG("Direct com : R-Block ACK received");
        break;
      } else {
        LOG_DBG("Direct com : R-Block NACK received");
        status = 0;
        continue;
      }
    
    }
    // S-Block handling //
    else if ((rx_buffer[0] & CLRC663_14443P4_PCB_BLOCK_MASK) == CLRC663_14443P4_PCB_S_BLOCK_ID) {
      if ((rx_buffer[0] & CLRC663_14443P4_PCB_S_BLOCK_CMD_MASK) == CLRC663_14443P4_PCB_S_BLOCK_DESELECT) {
        LOG_DBG("Direct com : S-Block DESELECT received");
        status = clrc663_indataexchange(dev, rx_buffer, 1, NULL, 0); // Send the ACK
        break;
      } else {
        LOG_ERR("Direct com : S-Block WTX not supported at the moment");
        status = CLRC663_COM_CODE_DATA_FORMAT_ERROR;
        break;
      }
    }
    // Invalid block //
    else {
      LOG_DBG("Direct com : PCB block type mismatch");
      status = CLRC663_COM_CODE_DATA_FORMAT_ERROR; // Try a R-Block NACK rule 4 of the ISO 14443-4
      continue;
    }
  }
  
  // Check the status of the communication
  if (status == 0) {
    LOG_DBG("Direct com : No data received from the PICC");
    return 0;
  } else if (status-skip > data_rx_len) {
    LOG_DBG("Direct com : Data too long for the reception from the PICC (max %hhd / receive %hhd)", data_rx_len, status-skip);
    status = CLRC663_COM_CODE_BUFFER_OVERFLOW;
  } else if (status > 0){
    status = status-skip;
    bytecpy(data_rx, rx_buffer+skip, status);
    LOG_HEXDUMP_DBG(data_rx, status, "Direct com : RAW data receive: ");
  }

  // Free the buffers
  k_free(tx_buffer);
  k_free(rx_buffer);
  
  return status;
}

/*! \brief Change the RF field state.
 *
 * This function change the RF field state.
 *
 * \param[in] dev The device structure.
 * \param[in] on The RF field state.
*/
static void clrc663_rf_field_control(const struct device *dev, bool on) {
  if (on) {
    clrc663_rf_field_on(dev);
  } else {
    clrc663_rf_field_off(dev);
  }
  k_sleep(K_MSEC(10)); // Wait for the RF field to be stable
}

/*! \brief Perform the calibration of the LPCD.
 *
 * This function performs the calibration of the LPCD.
 *
 * \param[in] dev The device structure.
 * \param[out] I_result The result of the I measurement.
 * \param[out] Q_result The result of the Q measurement.
 *
 * \return 0 on success, negative on error.
*/
static void clrc663_lpcd_calibration(const struct device *dev, uint8_t *I_result, uint8_t *Q_result) {
  clrc663_AN11145_start_IQ_measurement(dev);
  k_sleep(K_MSEC(10));
  clrc663_AN11145_stop_IQ_measurement(dev, I_result, Q_result);
}

/*! \brief Perform the activation of LPCD.
 *
 * This function performs the activation of LPCD.
 *
 * \param[in] dev The device structure.
 * \param[in] I_threshold The threshold of the I measurement.
 * \param[in] Q_threshold The threshold of the Q measurement.
 *
*/
static void clrc663_lpcd_activate(const struct device *dev, uint8_t I_mesured, uint8_t Q_mesured, uint8_t threshold) {
  clrc663_AN11145_activate_LPCD(dev, I_mesured, Q_mesured, threshold);
}

// TODO: Implement the following functions
// static int8_t clrc663_reset(const struct device *dev);
// static int8_t clrc663_powercontrol(const struct device *dev);

// API definition
static const struct nfc_reader_api clrc663_api = {
	.nfc_select = clrc663_card_select,
	.nfc_layer4_com = clrc663_iso14443_4_communication,
  .rf_field_control = clrc663_rf_field_control,
  .lpcd_calibration = clrc663_lpcd_calibration,
  .lpcd_activate = clrc663_lpcd_activate,
};

// ---------------------------------------------------------------------------
// Define CLRC663 Init (Device Tree multiple instances support)
// ---------------------------------------------------------------------------
#define CLRC663_SPI_MODE SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER | SPI_LOCK_ON

#define CLRC663_INIT(n) \
  static struct clrc663_data clrc663_data_##n = { \
    .protocol = CLRC663_PROTO_ISO14443A_106_MILLER_MANCHESTER, \
    .tx_DR = 0, \
    .rx_DR = 0, \
    .rf_field_on = false, \
    .ATS_received = false, \
    .layer4_activate = false, \
    .CID_support = false, \
    .NAD_support = false, \
    .PCB_block_number = CLRC663_14443P4_BLOCK_NUMBER_DEFAULT, \
    .CID = CLRC663_14443P4_CID_DEFAULT, \
    .transmit_max_frame_size = fsci_to_bytes[0], \
    .SFGT = CLRC663_14443P4_SFGT_DEFAULT, \
    .FWT = CLRC663_14443P4_FWT_DEFAULT, \
    .DSM_DRM_same = false, \
    .DSM = 0, \
    .DRM = 0, \
  };  \
    \
	static const struct clrc663_config clrc663_config_##n = { \
		.spi = SPI_DT_SPEC_INST_GET(n, CLRC663_SPI_MODE, 1),  \
		.shutdown = GPIO_DT_SPEC_INST_GET(n, shutdown_gpios), \
    IF_ENABLED(CONFIG_CLRC663_TRIGGER,  \
			(.irq = GPIO_DT_SPEC_INST_GET(n, irq_gpios),))  \
	};  \
    \
	DEVICE_DT_INST_DEFINE(n,  \
                        clrc663_init, \
                        NULL, \
                        &clrc663_data_##n,  \
                        &clrc663_config_##n,  \
                        POST_KERNEL,  \
                        CONFIG_CLRC663_INIT_PRIORITY,  \
                        &clrc663_api);

DT_INST_FOREACH_STATUS_OKAY(CLRC663_INIT)
