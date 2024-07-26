/*
 * Copyright (c) 2024 Johann Gilliéron
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CLRC663_H
#define CLRC663_H

#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/nfc.h>
#include "clrc663_defines.h"
#include <stdio.h>

// External interaction functions. (private functions)
int clrc663_SPI_transfer(const struct device *dev, uint8_t *tx_data, uint8_t *rx_data, uint8_t len, bool last_transfer);
void clrc663_poweroff(const struct device *dev);
void clrc663_poweron(const struct device *dev);

// Register interaction functions. (private functions)
uint8_t clrc663_read_reg(const struct device *dev, uint8_t reg);
void clrc663_write_reg(const struct device *dev, uint8_t reg, uint8_t value);
void clrc663_write_regs(const struct device *dev, const uint8_t reg, const uint8_t* values, uint8_t len);
void clrc663_write_fifo(const struct device *dev, const uint8_t* data, uint16_t len);
void clrc663_read_fifo(const struct device *dev, uint8_t* rx, uint16_t len);

// Command functions. (private functions)
void clrc663_cmd_soft_reset(const struct device *dev);
void clrc663_cmd_read_E2(const struct device *dev, uint16_t address, uint16_t length);
void clrc663_cmd_load_reg(const struct device *dev, uint16_t address, uint8_t regaddr, uint16_t length);
void clrc663_cmd_load_protocol(const struct device *dev, uint8_t rx, uint8_t tx);
void clrc663_cmd_transceive(const struct device *dev, const uint8_t* data, uint16_t len);
void clrc663_cmd_idle(const struct device *dev);
void clrc663_cmd_load_key_E2(const struct device *dev, uint8_t key_nr);
void clrc663_cmd_authenticate(const struct device *dev, uint8_t key_nr, uint8_t block_nr);
void clrc663_cmd_load_key(const struct device *dev, const uint8_t* key);

// Utility functions. (private functions)
void clrc663_rf_field_on(const struct device *dev);
void clrc663_rf_field_off(const struct device *dev);
void clrc663_init_registers(const struct device *dev);
uint8_t clrc663_get_version(const struct device *dev);
void clrc663_flush_fifo(const struct device *dev);
uint16_t clrc663_fifo_length(const struct device *dev);
void clrc663_clear_irq0(const struct device *dev);
void clrc663_clear_irq1(const struct device *dev);
uint8_t clrc663_irq0(const struct device *dev);
uint8_t clrc663_irq1(const struct device *dev);
uint8_t clrc663_transfer_E2_page(const struct device *dev, uint8_t* dest, uint8_t page);
uint8_t clrc663_error_name(const struct device *dev, uint8_t error, char* text_before_error);
void clrc663_load_predefined_protocol(const struct device *dev, uint8_t protocol);
uint16_t clrc663_irq_wait(const struct device *dev, uint8_t irq0_en, uint8_t irq1_en);
int16_t clrc663_indataexchange(const struct device *dev, uint8_t* tx_buffer, uint16_t tx_data_len, uint8_t* rx_buffer, uint16_t rx_data_len);

// Timer functions (private functions)
void clrc663_activate_timer(const struct device *dev, uint8_t timer, uint8_t active);
void clrc663_timer_set_control(const struct device *dev, uint8_t timer, uint8_t value);
void clrc663_timer_set_reload(const struct device *dev, uint8_t timer, uint16_t value);
void clrc663_timer_set_value(const struct device *dev, uint8_t timer, uint16_t value);
uint16_t clrc663_timer_get_value(const struct device *dev, uint8_t timer);
void clrc663_set_timout_timer(const struct device *dev, uint8_t timer, uint16_t value);

// From documentation (private functions)
void clrc663_AN11145_start_IQ_measurement(const struct device *dev);
void clrc663_AN11145_stop_IQ_measurement(const struct device *dev, uint8_t *Ires, uint8_t *Qres);
void clrc663_AN11145_activate_LPCD(const struct device *dev, uint8_t Imesured, uint8_t Qmesured, uint8_t threshold);
int8_t clrc663_AN11145_end_LPCD(const struct device *dev);
void clrc663_load_predefined_protocol(const struct device *dev, uint8_t protocol);

//  ISO 14443-3A/4 functions (private functions)
int16_t clrc663_iso14443a_REQA(const struct device *dev, uint8_t *atqa);
int16_t clrc663_iso14443a_WUPA(const struct device *dev, uint8_t *atqa);
int16_t clrc663_iso14443a_WUPA_REQA(const struct device *dev, uint8_t *atqa);
int16_t clrc663_iso14443p4_RATS(const struct device *dev, uint8_t *ats);
int16_t clrc663_iso14443p4_PPS(const struct device *dev);
uint8_t clrc663_iso14443p4_DESELECT(const struct device *dev);
int8_t clrc663_iso14443a_HLTA(const struct device *dev);

// API functions
const struct nfc_reader_api clrc663_api;
int clrc663_init(const struct device *dev);
static uint8_t clrc663_card_select(const struct device *dev, uint8_t* uid, uint8_t* sak, uint8_t* atqa, uint8_t* ats);
static int16_t clrc663_iso14443_4_communication(const struct device *dev, uint8_t* data_tx, uint16_t data_tx_len, uint8_t* data_rx, uint16_t data_rx_len);
static uint8_t clrc663_rf_field_control(const struct device *dev, bool on);
static uint8_t clrc663_lpcd_calibration(const struct device *dev, uint8_t *I_result, uint8_t *Q_result);
static uint8_t clrc663_lpcd_activate(const struct device *dev, uint8_t I_mesured, uint8_t Q_mesured, uint8_t threshold);

#endif // CLRC663_H

