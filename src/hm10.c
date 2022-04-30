#include "hm10.h"

// ===============================================================================
// Macros (Debug)
// ===============================================================================

#ifdef HM10_DEBUG
#define DEBUG_PRINTF pc_printf
#else
#define DEBUG_PRINTF(__X__, ...)
#endif

#define SHORT_TIMEOUT 3000 // in ms

// ===============================================================================
// Private functions
// ===============================================================================

// Reads a buffer until the carriage return
static uint16_t _readUntilCR(hm10_t *hm, uint16_t timeout) {
  uint32_t previous = millis();
  xstr_clear(hm->_buf);
  while (millis() - previous < timeout) {
    if (hm->uart_obj->available() > 0) {
      int c = hm->uart_obj->read();
      if (c == '\r') { break; }
      if (!xstr_push(hm->_buf, c)) { break; }
    }
  }
  return xstr_len(hm->_buf);
}

// Check if response is equals to the one specify
static bool _expectResponse(hm10_t *hm, const char *expected_response, uint16_t timeout) {
  DEBUG_PRINTF("  => expectedResponse: %s\n", expected_response);
  if (_readUntilCR(hm, timeout) > 0) {
    DEBUG_PRINTF("  => Received (%s)", hm->_buf->ptr);
    if (xstr_index(hm->_buf, expected_response, 0) >= 0) {
      DEBUG_PRINTF(" found match!\n");
      return true;
    }
    DEBUG_PRINTF(" not match...!\n");
    return false;
  }
  DEBUG_PRINTF("  => TIMEOUT!\n");
  return false;
}

void _sendAtCommand(hm10_t *hm, const char *at_cmd) {
  DEBUG_PRINTF("=> Command (%s)\n", at_cmd);
  // send at command
  xfprint(hm->uart_obj->write, at_cmd);
  // Flush uart rx
  hm10_flushRx(hm);
  // Finish the command
  hm->uart_obj->write('\r');
  hm->uart_obj->write('\n');
  // clear the buffer
  xstr_clear(hm->_buf);
}

void _wakeUp(hm10_t *hm) {
  delay(500);
  _sendAtCommand(hm, "1231212312"); // just wake up
  xstr_clear(hm->_buf);
}

// ===============================================================
// Public functions
// ===============================================================

// Initialize the HM10 module
void hm10_init(hm10_t *hm) {
  gpio_mode(hm->state_pin, INPUT, PULLDOWN, SPEED_HIGH);
  gpio_mode(hm->en_pin, OUTPUT_PP, NOPULL, SPEED_HIGH);
  // keep the "en" pin at HIGH state to avoid disturbances.
  gpio_set(hm->en_pin);
  delay(100);

  hm10_reset(hm);
}

// Performs a software reset
void hm10_reset(hm10_t *hm) {
  _sendAtCommand(hm, "AT+RESET");
  _expectResponse(hm, "OK", SHORT_TIMEOUT);
  _wakeUp(hm);
}

// Set HM10 local name
bool hm10_setName(hm10_t *hm, const uint8_t *name, uint8_t nameLen) {
  xstr_t b = XSTR_INITIALIZER(30);
  xsprintf(&b, "AT+NAME");
  for (uint8_t i = 0; i < nameLen; i++) {
    xstr_push(&b, name[i]);
  }
  xstr_push(&b, '\0');
  _sendAtCommand(hm, (const char *) b.ptr);
  xstr_clear(&b);
  xsprintf(&b, "+NAME=");
  for (uint8_t i = 0; i < nameLen; i++) {
    xstr_push(&b, name[i]);
  }
  xstr_push(&b, '\0');
  bool ret = _expectResponse(hm, (const char *) b.ptr, SHORT_TIMEOUT);
  _wakeUp(hm);
  return ret;
}

// Set HM10 role (Peripheral or Central role)
bool hm10_setRole(hm10_t *hm, uint8_t role) {
  bool ret = false;
  if (role == HM10_ROLE_PERIPH) {
    _sendAtCommand(hm, "AT+ROLE0");
    ret = _expectResponse(hm, "+ROLE=0", SHORT_TIMEOUT);
  } else {
    _sendAtCommand(hm, "AT+ROLE1");
    ret = _expectResponse(hm, "+ROLE=1", SHORT_TIMEOUT);
  }
  _wakeUp(hm);
  return ret;
}

// Set HM10 work type (AT+IMME)
bool hm10_setWorkType(hm10_t *hm, uint8_t workType) {
  bool ret = false;
  if (workType == HM10_WORKTYPE_IMMEDIATELY) {
    _sendAtCommand(hm, "AT+IMME0");
    ret = _expectResponse(hm, "OK+Set:0", SHORT_TIMEOUT);
  } else {
    _sendAtCommand(hm, "AT+IMME1");
    ret = _expectResponse(hm, "OK+Set:1", SHORT_TIMEOUT);
  }
  _wakeUp(hm);
  return ret;
}

// Check if the device is connected.
bool hm10_isConnected(hm10_t *hm) {
  return gpio_read(hm->state_pin) == HIGH;
}
// Send bytes (max 20 bytes)
void hm10_send(hm10_t *hm, const uint8_t *buf, uint8_t nbytes) {
  if (!hm10_isConnected(hm)) {
    DEBUG_PRINTF("Can't send data if device is not connected");
    return;
  }
  for (uint8_t i = 0; i < nbytes; i++) {
    hm->uart_obj->write(buf[i]);
  }
}

// Returns how many unread bytes has the HM10.
uint16_t hm10_availableData(hm10_t *hm) {
  return hm->uart_obj->available();
}

// Returns one byte if there is data available in HM10.
int hm10_read(hm10_t *hm) {
  return hm->uart_obj->read();
}

// Discards all data in rx buffer
void hm10_flushRx(hm10_t *hm) {
  while (hm->uart_obj->available() > 0) {
    hm->uart_obj->read();
  }
}

// Receives N bytes. It returns the real quantity of bytes read.
int hm10_receive(hm10_t *hm, uint8_t *rcvBuf, uint8_t nbytes, uint16_t timeout) {
  if (!hm10_isConnected(hm)) {
    DEBUG_PRINTF("Can't receive data if device is not connected");
    return -1;
  }
  memset(rcvBuf, 0, nbytes);
  uint8_t cnt       = 0;
  uint32_t previous = millis();
  while (cnt < nbytes && (millis() - previous < timeout)) {
    if (hm10_availableData(hm) > 0) {
      int c = hm10_read(hm);
      if (c == -1) { break; }
      rcvBuf[cnt] = c;
      cnt++;
    }
  }
  return cnt;
}
