/*
 * Copyright (c) 2024 Johann Gilliéron
 * Based on the work of: 2024 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_NFC_READER_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_NFC_READER_H_
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif


/** @cond INTERNAL_HIDDEN */

typedef uint8_t (*nfc_rf_field_control_t)(const struct device *dev, bool mode);
typedef uint8_t (*nfc_lpcd_calibration_t)(const struct device *dev, uint8_t *I_measured, uint8_t *Q_measured);
typedef uint8_t (*nfc_lpcd_activate_t)(const struct device *dev, uint8_t I_measured, uint8_t Q_measured, uint8_t threshold);
typedef uint8_t (*nfc_card_select_t)(const struct device *dev, uint8_t* uid, uint8_t* sak, uint8_t* atqa, uint8_t* ats);
typedef int16_t (*nfc_card_iso14443_4_com_t)(const struct device *dev, uint8_t* data_tx, uint8_t data_tx_len, uint8_t* data_rx, uint8_t data_rx_len);

__subsystem struct nfc_reader_api {
	nfc_card_select_t nfc_select;
	nfc_card_iso14443_4_com_t nfc_layer4_com;
	nfc_rf_field_control_t rf_field_control;
	nfc_lpcd_calibration_t lpcd_calibration;
	nfc_lpcd_activate_t lpcd_activate;
};

/** @endcond */
/**
 * @brief Detect if a card is in the field and select it (read its UID, SAK, ATQA and ATS if PICC is compatible).
 *
 * @param dev nfc reader instance.
 * @param uid buffer for the UID of the tag (10 bytes)
 * @param sak buffer for the SAK of the tag	(1 bytes)
 * @param atqa buffer for the ATQA of the tag (2 bytes)
 * @param ats buffer for the ATS answer to RATS (16 bytes)
 *
 * @retval 0 on failure.
 * @retval uid length on success.
 */
__syscall uint8_t nfc_card_select(const struct device *dev, uint8_t* uid, uint8_t* sak, uint8_t* atqa, uint8_t* ats);

static inline uint8_t z_impl_nfc_card_select(const struct device *dev, uint8_t* uid, uint8_t* sak, uint8_t* atqa, uint8_t* ats)
{
	const struct nfc_reader_api *api = (const struct nfc_reader_api *)dev->api;
	return api->nfc_select(dev, uid, sak, atqa, ats);
}

/**
 * @brief Direct communication between uC and the NFC tag with protocol ISO14443-4 if compatible.
 * @param dev nfc reader instance.
 * @param data_tx buffer for the data to send
 * @param data_tx_len length of the data to send
 * @param data_rx buffer for the data to receive
 * @param data_rx_len length MAX of the data to receive
 *
 * @retval Number of byte receive on success.
 * @retval negative in case of failure.
 */
__syscall int16_t nfc_card_iso14443_4_com(const struct device *dev, uint8_t* data_tx, uint8_t data_tx_len, uint8_t* data_rx, uint8_t data_rx_len);

static inline int16_t z_impl_nfc_card_iso14443_4_com(const struct device *dev, uint8_t* data_tx, uint8_t data_tx_len, uint8_t* data_rx, uint8_t data_rx_len)
{
	const struct nfc_reader_api *api = (const struct nfc_reader_api *)dev->api;
	return api->nfc_layer4_com(dev, data_tx, data_tx_len, data_rx, data_rx_len);
}

/**
 * @brief Control the RF field.
 *
 * @param dev nfc reader instance.
 * @param mode true to enable the RF field, false to disable it.
 */
__syscall uint8_t nfc_rf_field_control(const struct device *dev, bool mode);

static inline uint8_t z_impl_nfc_rf_field_control(const struct device *dev, bool mode)
{
	const struct nfc_reader_api *api = (const struct nfc_reader_api *)dev->api;
	return api->rf_field_control(dev, mode);
}

/**
 * @brief Calibrate the LPCD.
 *
 * @param dev nfc reader instance.
 * @param I_measured pointer to the I measured value.
 * @param Q_measured pointer to the Q measured value.
 */
__syscall uint8_t nfc_lpcd_calibration(const struct device *dev, uint8_t *I_measured, uint8_t *Q_measured);

static inline uint8_t z_impl_nfc_lpcd_calibration(const struct device *dev, uint8_t *I_measured, uint8_t *Q_measured)
{
	const struct nfc_reader_api *api = (const struct nfc_reader_api *)dev->api;
	return api->lpcd_calibration(dev, I_measured, Q_measured);
}

/**
 * @brief Activate the LPCD.
 *
 * @param dev nfc reader instance.
 * @param I_threshold I threshold value.
 * @param Q_threshold Q threshold value.
 */
__syscall uint8_t nfc_lpcd_activate(const struct device *dev, uint8_t I_measured, uint8_t Q_measured, uint8_t threshold);

static inline uint8_t z_impl_nfc_lpcd_activate(const struct device *dev, uint8_t I_measured, uint8_t Q_measured, uint8_t threshold)
{
	const struct nfc_reader_api *api = (const struct nfc_reader_api *)dev->api;
	return api->lpcd_activate(dev, I_measured, Q_measured, threshold);
}


#ifdef __cplusplus
}
#endif

#include <syscalls/nfc.h>
#endif  // ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_NFC_READER_H_
