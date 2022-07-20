/*!
 * @file Adafruit_Fingerprint.c
 *
 * @mainpage Adafruit Fingerprint Sensor Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for our optical Fingerprint sensor
 *
 * Designed specifically to work with the Adafruit Fingerprint sensor
 * ----> http://www.adafruit.com/products/751
 *
 * These displays use TTL Serial to communicate, 2 pins are required to
 * interface
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 *
 */

#include "include/Adafruit_Fingerprint/conf.h"
#include "include/Adafruit_Fingerprint.h"
#include <stdarg.h>
#include <string.h>

#define FINGERPRINT_DEBUG

static void writeStructuredPacket(Adafruit_Fingerprint_t *fgrPrint,
                                  const Adafruit_Fingerprint_Packet_t *p);
static uint8_t getStructuredPacket(Adafruit_Fingerprint_t *fgrPrint,
                                   Adafruit_Fingerprint_Packet_t *p,
                                   uint16_t timeout);
static const uint8_t default_address[4] = {0xFF, 0xFF, 0xFF, 0xFF};

/*!
 * @brief Gets the command packet
 */
#define GET_CMD_PACKET(fgrPrint, ...)                                         \
  Adafruit_Fingerprint_Packet_t *packet = &(fgrPrint)->tmpPacket;             \
  uint8_t                       data[]  = {__VA_ARGS__};                      \
  packet->type = FINGERPRINT_COMMANDPACKET;                                   \
  packet->length = sizeof(data);                                              \
  memcpy(&packet->data[0], data, packet->length);                             \
  writeStructuredPacket(fgrPrint, packet);                                    \
  if (getStructuredPacket(fgrPrint, packet, 5000) != FINGERPRINT_OK)          \
    goto end;                                                                 \
  if (packet->type != FINGERPRINT_ACKPACKET)                                  \
    goto end;

/*!
 * @brief Sends the command packet
 */
#define SEND_CMD_PACKET(fgrPrint, ...)                                        \
  uint8_t status = FINGERPRINT_PACKETRECIEVEERR;                              \
  if (AFGR_MutexLock(fgrPrint, 2000) != 0) {                                  \
    status = FINGERPRINT_TIMEOUT;                                             \
    goto end;                                                                 \
  }                                                                           \
  GET_CMD_PACKET(fgrPrint, __VA_ARGS__);                                      \
  status = packet->data[0];                                                   \
  end:                                                                        \
  AFGR_MutexUnlock(fgrPrint);                                                 \
  return status;

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

void AFGR_Init(Adafruit_Fingerprint_t *fgrPrint,
               uint8_t *txBuffer, uint16_t txBufferSize)
{
  fgrPrint->txBuffer = txBuffer;
  fgrPrint->txBufferSize = txBufferSize;
  fgrPrint->theAddress = 0xFFFFFFFFU;

  memcpy(&fgrPrint->tmpPacket.address[0], default_address, sizeof(fgrPrint->tmpPacket.address));
}


/**************************************************************************/
/*!
    @brief  Verifies the sensors' access password (default password is
   0x0000000). A good way to also check if the sensors is active and responding
    @returns True if password is correct
*/
/**************************************************************************/
uint8_t AFGR_VerifyPassword(Adafruit_Fingerprint_t *fgrPrint)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_VERIFYPASSWORD,
                  (uint8_t)(fgrPrint->thePassword >> 24),
                  (uint8_t)(fgrPrint->thePassword >> 16),
                  (uint8_t)(fgrPrint->thePassword >> 8),
                  (uint8_t)(fgrPrint->thePassword & 0xFF));
}


/**************************************************************************/
/*!
    @brief  Get the sensors parameters, fills in the member variables
    status_reg, system_id, capacity, security_level, device_addr, packet_len
    and baud_rate
    @returns True if password is correct
*/
/**************************************************************************/
uint8_t AFGR_GetParameters(Adafruit_Fingerprint_t *fgrPrint)
{
  uint8_t status = FINGERPRINT_PACKETRECIEVEERR;

  if (AFGR_MutexLock(fgrPrint, 2000) != 0) {
    status = FINGERPRINT_TIMEOUT;
    goto end;
  }

  GET_CMD_PACKET(fgrPrint, FINGERPRINT_READSYSPARAM);

  fgrPrint->status_reg      = ((uint16_t)packet->data[1] << 8) | packet->data[2];
  fgrPrint->system_id       = ((uint16_t)packet->data[3] << 8) | packet->data[4];
  fgrPrint->capacity        = ((uint16_t)packet->data[5] << 8) | packet->data[6];
  fgrPrint->security_level  = ((uint16_t)packet->data[7] << 8) | packet->data[8];
  fgrPrint->device_addr     = ((uint32_t)packet->data[9] << 24)  |
                              ((uint32_t)packet->data[10] << 16) |
                              ((uint32_t)packet->data[11] << 8)  |
                                (uint32_t)packet->data[12];
  fgrPrint->packet_len = ((uint16_t)packet->data[13] << 8) | packet->data[14];
  if (fgrPrint->packet_len == 0) {
    fgrPrint->packet_len = 32;
  } else if (fgrPrint->packet_len == 1) {
    fgrPrint->packet_len = 64;
  } else if (fgrPrint->packet_len == 2) {
    fgrPrint->packet_len = 128;
  } else if (fgrPrint->packet_len == 3) {
    fgrPrint->packet_len = 256;
  }
  fgrPrint->baud_rate = (((uint16_t)packet->data[15] << 8) | packet->data[16]) * 9600;

  status = packet->data[0];

  end:
  AFGR_MutexUnlock(fgrPrint);
  return status;
}


/**************************************************************************/
/*!
    @brief   Ask the sensor to take an image of the finger pressed on surface
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_NOFINGER</code> if no finger detected
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_IMAGEFAIL</code> on imaging error
*/
/**************************************************************************/
uint8_t AFGR_GetImage(Adafruit_Fingerprint_t *fgrPrint)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_GETIMAGE);
}


/**************************************************************************/
/*!
    @brief   Ask the sensor to convert image to feature template
    @param slot Location to place feature template (put one in 1 and another in
   2 for verification to create model)
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_IMAGEMESS</code> if image is too messy
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_FEATUREFAIL</code> on failure to identify
   fingerprint features
    @returns <code>FINGERPRINT_INVALIDIMAGE</code> on failure to identify
   fingerprint features
*/
uint8_t AFGR_Image2Tz(Adafruit_Fingerprint_t *fgrPrint, uint8_t slot)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_IMAGE2TZ, slot);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to take two print feature template and create a
   model
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_ENROLLMISMATCH</code> on mismatch of fingerprints
*/
uint8_t AFGR_CreateModel(Adafruit_Fingerprint_t *fgrPrint)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_REGMODEL);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to store the calculated model for later matching
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
   to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t AFGR_StoreModel(Adafruit_Fingerprint_t *fgrPrint, uint16_t id)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_STORE, 0x01, (uint8_t)(id >> 8),
                  (uint8_t)(id & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to load a fingerprint model from flash into buffer 1
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t AFGR_LoadModel(Adafruit_Fingerprint_t *fgrPrint, uint16_t id)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_LOAD, 0x01, (uint8_t)(id >> 8),
                  (uint8_t)(id & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to transfer 256-byte fingerprint template from the
   buffer to the UART
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/

uint8_t AFGR_GetModel(Adafruit_Fingerprint_t *fgrPrint)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_UPLOAD, 0x01);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to delete a model in memory
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
   to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t AFGR_DeleteModel(Adafruit_Fingerprint_t *fgrPrint, uint16_t id)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_DELETE, (uint8_t)(id >> 8),
                  (uint8_t)(id & 0xFF), 0x00, 0x01);
}


/**************************************************************************/
/*!
    @brief   Ask the sensor to delete ALL models in memory
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
   to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t AFGR_EmptyDatabase(Adafruit_Fingerprint_t *fgrPrint)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_EMPTY);
}


/**************************************************************************/
/*!
    @brief   Ask the sensor to search the current slot 1 fingerprint features to
   match saved templates. The matching location is stored in <b>fingerID</b> and
   the matching confidence in <b>confidence</b>
    @returns <code>FINGERPRINT_OK</code> on fingerprint match success
    @returns <code>FINGERPRINT_NOTFOUND</code> no match made
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t AFGR_FingerFastSearch(Adafruit_Fingerprint_t *fgrPrint)
{
  uint8_t status = FINGERPRINT_PACKETRECIEVEERR;

  if (AFGR_MutexLock(fgrPrint, 2000) != 0) {
    status = FINGERPRINT_TIMEOUT;
    goto end;
  }

  // high speed search of slot #1 starting at page 0x0000 and page #0x00A3
  GET_CMD_PACKET(fgrPrint, FINGERPRINT_HISPEEDSEARCH, 0x01, 0x00, 0x00, 0x00, 0xA3);
  fgrPrint->fingerID = 0xFFFF;
  fgrPrint->confidence = 0xFFFF;

  fgrPrint->fingerID = packet->data[1];
  fgrPrint->fingerID <<= 8;
  fgrPrint->fingerID |= packet->data[2];

  fgrPrint->confidence = packet->data[3];
  fgrPrint->confidence <<= 8;
  fgrPrint->confidence |= packet->data[4];

  status = packet->data[0];

  end:
  AFGR_MutexUnlock(fgrPrint);
  return status;
}


/**************************************************************************/
/*!
    @brief   Ask the sensor to search the current slot fingerprint features to
   match saved templates. The matching location is stored in <b>fingerID</b> and
   the matching confidence in <b>confidence</b>
   @param slot The slot to use for the print search, defaults to 1
    @returns <code>FINGERPRINT_OK</code> on fingerprint match success
    @returns <code>FINGERPRINT_NOTFOUND</code> no match made
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t AFGR_FingerSearch(Adafruit_Fingerprint_t *fgrPrint, uint8_t slot)
{
  uint8_t status = FINGERPRINT_PACKETRECIEVEERR;

  if (AFGR_MutexLock(fgrPrint, 2000) != 0) {
    status = FINGERPRINT_TIMEOUT;
    goto end;
  }

  // search of slot starting thru the capacity
  GET_CMD_PACKET(fgrPrint, FINGERPRINT_SEARCH, slot, 0x00, 0x00, (uint8_t)(fgrPrint->capacity >> 8),
                 (uint8_t)(fgrPrint->capacity & 0xFF));

  fgrPrint->fingerID = 0xFFFF;
  fgrPrint->confidence = 0xFFFF;

  fgrPrint->fingerID = packet->data[1];
  fgrPrint->fingerID <<= 8;
  fgrPrint->fingerID |= packet->data[2];

  fgrPrint->confidence = packet->data[3];
  fgrPrint->confidence <<= 8;
  fgrPrint->confidence |= packet->data[4];

  status = packet->data[0];

  end:
  AFGR_MutexUnlock(fgrPrint);
  return status;
}


/**************************************************************************/
/*!
    @brief   Ask the sensor for the number of templates stored in memory. The
   number is stored in <b>templateCount</b> on success.
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t AFGR_GetTemplateCount(Adafruit_Fingerprint_t *fgrPrint)
{
  uint8_t status = FINGERPRINT_PACKETRECIEVEERR;

  if (AFGR_MutexLock(fgrPrint, 2000) != 0) {
    status = FINGERPRINT_TIMEOUT;
    goto end;
  }

  GET_CMD_PACKET(fgrPrint, FINGERPRINT_TEMPLATECOUNT);

  fgrPrint->templateCount = packet->data[1];
  fgrPrint->templateCount <<= 8;
  fgrPrint->templateCount |= packet->data[2];

  status = packet->data[0];

  end:
  AFGR_MutexUnlock(fgrPrint);
  return status;
}

/**************************************************************************/
/*!
    @brief   Set the password on the sensor (future communication will require
   password verification so don't forget it!!!)
    @param   password 32-bit password code
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t AFGR_SetPassword(Adafruit_Fingerprint_t *fgrPrint, uint32_t password)
{
  SEND_CMD_PACKET(fgrPrint, FINGERPRINT_SETPASSWORD,
                  (uint8_t)(password >> 24),
                  (uint8_t)(password >> 16),
                  (uint8_t)(password >> 8),
                  (uint8_t)(password & 0xFF));
}


/**************************************************************************/
/*!
    @brief   Control the built in LED
    @param on True if you want LED on, False to turn LED off
    @returns <code>FINGERPRINT_OK</code> on success
*/
/**************************************************************************/
uint8_t AFGR_LEDcontrol(Adafruit_Fingerprint_t *fgrPrint, uint8_t on)
{
  uint8_t                       status  = FINGERPRINT_PACKETRECIEVEERR;
  Adafruit_Fingerprint_Packet_t *packet = &(fgrPrint)->tmpPacket;

  if (AFGR_MutexLock(fgrPrint, 2000) != 0) {
    status = FINGERPRINT_TIMEOUT;
    goto end;
  }

  packet->type = FINGERPRINT_COMMANDPACKET;
  packet->length = 1;

  if (on) {
    packet->data[0] = FINGERPRINT_LEDON;
  } else {
    packet->data[0] = FINGERPRINT_LEDOFF;
  }

  writeStructuredPacket(fgrPrint, packet);
  if (getStructuredPacket(fgrPrint, packet, 5000) != FINGERPRINT_OK)
    goto end;
  if (packet->type != FINGERPRINT_ACKPACKET)
    goto end;

  status = packet->data[0];

  end:
  AFGR_MutexUnlock(fgrPrint);
  return status;
}


__attribute__((weak)) uint8_t AFGR_MutexLock(Adafruit_Fingerprint_t *fgrPrint, uint16_t timeout)
{
  return 0;
}


__attribute__((weak)) uint8_t AFGR_MutexUnlock(Adafruit_Fingerprint_t *fgrPrint)
{
  return 0;
}


/**************************************************************************/
/*!
    @brief   Helper function to process a packet and send it over UART to the
   sensor
    @param   packet A structure containing the bytes to transmit
*/
/**************************************************************************/
static void writeStructuredPacket(Adafruit_Fingerprint_t *fgrPrint,
                                  const Adafruit_Fingerprint_Packet_t *packet)
{
  uint16_t i = 0;
  uint8_t *buffer = fgrPrint->txBuffer;

  if (fgrPrint->txBuffer == 0) return;
  if (fgrPrint->txBufferSize < 9) return;

  *(buffer + (i++)) = (uint8_t)((AFGR_START_CODE >> 8) & 0xFF);
  *(buffer + (i++)) = (uint8_t)(AFGR_START_CODE & 0xFF);
  *(buffer + (i++)) = packet->address[0];
  *(buffer + (i++)) = packet->address[1];
  *(buffer + (i++)) = packet->address[2];
  *(buffer + (i++)) = packet->address[3];
  *(buffer + (i++)) = packet->type;

  uint16_t wire_length = packet->length + 2;
  *(buffer + (i++)) = (uint8_t)((wire_length >> 8)& 0xFF);
  *(buffer + (i++)) = (uint8_t)(wire_length & 0xFF);

  uint16_t sum = ((wire_length) >> 8) + ((wire_length)&0xFF) + packet->type;
  for (uint16_t j = 0; j < packet->length; j++) {
    if (i >= fgrPrint->txBufferSize) {
      i = 0;
      if (fgrPrint->transmitBytes != 0) {
        fgrPrint->transmitBytes(buffer, i);
      }
    }
    *(buffer + (i++)) = packet->data[j];
    sum += packet->data[j];
  }

  if (i >= (fgrPrint->txBufferSize-2)) {
    i = 0;
    if (fgrPrint->transmitBytes != 0) {
      fgrPrint->transmitBytes(buffer, i);
    }
  }
  *(buffer + (i++)) = (uint8_t)(sum >> 8);
  *(buffer + (i++)) = (uint8_t)(sum & 0xFF);

  if (fgrPrint->transmitBytes != 0) {
    fgrPrint->transmitBytes(buffer, i);
  }

  return;
}

/**************************************************************************/
/*!
    @brief   Helper function to receive data over UART from the sensor and
   process it into a packet
    @param   packet A structure containing the bytes received
    @param   timeout how many milliseconds we're willing to wait
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_TIMEOUT</code> or
   <code>FINGERPRINT_BADPACKET</code> on failure
*/
/**************************************************************************/
static uint8_t getStructuredPacket(Adafruit_Fingerprint_t *fgrPrint,
                                   Adafruit_Fingerprint_Packet_t *packet,
                                   uint16_t timeout)
{
  uint8_t byte;
  uint16_t idx = 0;
  uint16_t start_code = 0;
  uint32_t tick = AFGR_GetTick();

  if (fgrPrint->readByte == 0 || fgrPrint->isAvailable == 0 || tick == 0)
    return FINGERPRINT_TIMEOUT;

  while (1) {
    while (!fgrPrint->isAvailable()) {
      if ((AFGR_GetTick() - tick) >= timeout) {
        return FINGERPRINT_TIMEOUT;
      }
      AFGR_Delay(1);
    }
    byte = fgrPrint->readByte();
    switch (idx) {
    case 0:
      if (byte != (AFGR_START_CODE >> 8))
        continue;
      start_code = (uint16_t)byte << 8;
      break;
    case 1:
      start_code |= byte;
      if (start_code != AFGR_START_CODE)
        return FINGERPRINT_BADPACKET;
      break;
    case 2:
    case 3:
    case 4:
    case 5:
      packet->address[idx - 2] = byte;
      break;
    case 6:
      packet->type = byte;
      break;
    case 7:
      packet->length = (uint16_t)byte << 8;
      break;
    case 8:
      packet->length |= byte;
      break;
    default:
      packet->data[idx - 9] = byte;
      if ((idx - 8) == packet->length) {
        return FINGERPRINT_OK;
      }
      break;
    }
    idx++;
  }
  // Shouldn't get here so...
  return FINGERPRINT_BADPACKET;
}
