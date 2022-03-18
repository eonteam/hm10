/**
 ******************************************************************************
 * @file    hm10.h
 * @author  EonTeam
 * @version V1.0.0
 * @date    2022
 ******************************************************************************
 */

#ifndef __HM10_H_
#define __HM10_H_

#include "eonOS.h"

// ===============================================================================
// Macros
// ===============================================================================

#define HM10_ROLE_PERIPH          (uint8_t)(0x00)
#define HM10_ROLE_CENTRAL         (uint8_t)(0x01)
#define HM10_WORKTYPE_IMMEDIATELY (uint8_t)(0x00)
#define HM10_WORKTYPE_ON_COMMAND  (uint8_t)(0x01)

// ===============================================================================
// Types
// ===============================================================================

typedef struct {
  const uart_obj_t *uart_obj;
  pin_t en_pin;
  pin_t state_pin;
  xstr_t *_buf; // internal buffer must be initialized when defining this
} hm10_t;

// ===============================================================================
// Functions
// ===============================================================================

// Initialize the HM10 module
void hm10_init(hm10_t *hm);
// Performs a software reset
void hm10_reset(hm10_t *hm);
// Set HM10 local name
bool hm10_setName(hm10_t *hm, const uint8_t *name, uint8_t nameLen);
// Set HM10 role (Peripheral or Central role)
bool hm10_setRole(hm10_t *hm, uint8_t role);
// Set HM10 work type (AT+IMME)
bool hm10_setWorkType(hm10_t *hm, uint8_t workType);
// Check if the device is connected.
bool hm10_isConnected(hm10_t *hm);
// Send bytes (max 20 bytes)
void hm10_send(hm10_t *hm, uint8_t *buf, uint8_t nbytes);
// Returns how many unread bytes has the HM10.
uint16_t hm10_availableData(hm10_t *hm);
// Returns one byte if there is data available in HM10.
int hm10_read(hm10_t *hm);
// Discards all data in rx buffer
void hm10_flushRx(hm10_t *hm);
// Receives N bytes. It returns the real quantity of bytes read.
int hm10_receive(hm10_t *hm, uint8_t *rcvBuf, uint8_t nbytes, uint16_t timeout);

#endif