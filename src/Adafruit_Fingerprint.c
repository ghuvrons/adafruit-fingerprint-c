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


static uint8_t mutexLock(uint32_t timeout);
static uint8_t mutexUnlock(void);
static uint8_t sendCommand(Fingerprint_t*, uint8_t command, uint8_t *data, uint16_t length);
static void sendPacket(Fingerprint_t*, const Fingerprint_Packet_t*);
static uint8_t getResponse(Fingerprint_t*, Fingerprint_Packet_t*, uint32_t timeout);
static uint16_t uint16ToBigEndian(uint16_t);
static uint32_t uint32ToBigEndian(uint32_t);

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

uint8_t AFGR_Init(Fingerprint_t *fgrPrint,
                  uint8_t *txBuffer, uint16_t txBufferSize,
                  uint8_t *rxBuffer, uint16_t rxBufferSize)
{
  if (fgrPrint->delay == 0
      || fgrPrint->getTick == 0
      || fgrPrint->write == 0
      || fgrPrint->read == 0)
  {
    return FINGERPRINT_ERROR;
  }

  if (fgrPrint->mutexLock == 0) fgrPrint->mutexLock = mutexLock;
  if (fgrPrint->mutexUnlock == 0) fgrPrint->mutexUnlock = mutexUnlock;

  fgrPrint->txBuffer = txBuffer;
  fgrPrint->txBufferSize = txBufferSize;
  fgrPrint->rxBuffer = rxBuffer;
  fgrPrint->rxBufferSize = rxBufferSize;
  
  return FINGERPRINT_OK;
}


/**************************************************************************/
/*!
    @brief  Verifies the sensors' access password (default password is
   0x0000000). A good way to also check if the sensors is active and responding
    @returns True if password is correct
*/
/**************************************************************************/
uint8_t AFGR_VerifyPassword(Fingerprint_t *fgrPrint)
{
  uint32_t bigEndianPassword = uint32ToBigEndian(fgrPrint->thePassword);
  return sendCommand(fgrPrint, FINGERPRINT_VERIFYPASSWORD, (uint8_t*) &bigEndianPassword, sizeof(uint32_t));
}


/**************************************************************************/
/*!
    @brief  Get the sensors parameters, fills in the member variables
    status_reg, system_id, capacity, security_level, device_addr, packet_len
    and baud_rate
    @returns True if password is correct
*/
/**************************************************************************/
uint8_t AFGR_GetParameters(Fingerprint_t *fgrPrint)
{
  uint8_t status;
  Fingerprint_Packet_t *packet = &(fgrPrint)->packet;

  status = sendCommand(fgrPrint, FINGERPRINT_READSYSPARAM, 0, 0);
  if (status != FINGERPRINT_OK) return status;

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
uint8_t AFGR_GetImage(Fingerprint_t *fgrPrint)
{
  return sendCommand(fgrPrint, FINGERPRINT_GETIMAGE, 0, 0);
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
uint8_t AFGR_Image2Tz(Fingerprint_t *fgrPrint, uint8_t slot)
{
  return sendCommand(fgrPrint, FINGERPRINT_IMAGE2TZ, slot, 1);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to take two print feature template and create a
   model
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_ENROLLMISMATCH</code> on mismatch of fingerprints
*/
uint8_t AFGR_CreateModel(Fingerprint_t *fgrPrint)
{
  return sendCommand(fgrPrint, FINGERPRINT_REGMODEL, 0, 0);
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
uint8_t AFGR_StoreModel(Fingerprint_t *fgrPrint, uint16_t id)
{
  uint8_t data[] = {
    0x01,
    (uint8_t)(id >> 8),
    (uint8_t)(id & 0xFF)
  };

  return sendCommand(fgrPrint, FINGERPRINT_STORE, data, sizeof(data));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to load a fingerprint model from flash into buffer 1
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t AFGR_LoadModel(Fingerprint_t *fgrPrint, uint16_t id)
{
  uint8_t data[] = {
    0x01,
    (uint8_t)(id >> 8),
    (uint8_t)(id & 0xFF)
  };

  return sendCommand(fgrPrint, FINGERPRINT_LOAD, data, sizeof(data));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to transfer 256-byte fingerprint template from the
   buffer to the UART
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/

uint8_t AFGR_GetModel(Fingerprint_t *fgrPrint)
{
  uint8_t data = 0x01;
  return sendCommand(fgrPrint, FINGERPRINT_UPLOAD, &data, 1);
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
uint8_t AFGR_DeleteModel(Fingerprint_t *fgrPrint, uint16_t id)
{
  uint8_t data[] = {
    (uint8_t)(id >> 8),
    (uint8_t)(id & 0xFF), 
    0x00, 
    0x01
  };
  
  return sendCommand(fgrPrint, FINGERPRINT_DELETE, data, sizeof(data));
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
uint8_t AFGR_EmptyDatabase(Fingerprint_t *fgrPrint)
{
  return sendCommand(fgrPrint, FINGERPRINT_EMPTY, 0, 0);
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
uint8_t AFGR_FingerFastSearch(Fingerprint_t *fgrPrint)
{
  uint8_t status;
  Fingerprint_Packet_t *packet = &(fgrPrint)->packet;
  uint8_t data[] = {
    0x01, 0x00, 0x00, 0x00, 0xA3
  };

  // high speed search of slot #1 starting at page 0x0000 and page #0x00A3
  status = sendCommand(fgrPrint, FINGERPRINT_HISPEEDSEARCH, data, sizeof(data));
  if (status != FINGERPRINT_OK) return status;

  fgrPrint->fingerID = 0xFFFF;
  fgrPrint->confidence = 0xFFFF;

  fgrPrint->fingerID = packet->data[1];
  fgrPrint->fingerID <<= 8;
  fgrPrint->fingerID |= packet->data[2];

  fgrPrint->confidence = packet->data[3];
  fgrPrint->confidence <<= 8;
  fgrPrint->confidence |= packet->data[4];

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
uint8_t AFGR_FingerSearch(Fingerprint_t *fgrPrint, uint8_t slot)
{
  uint8_t status;
  Fingerprint_Packet_t *packet = &(fgrPrint)->packet;

  uint8_t data[] = {
    slot, 
    0x00, 
    0x00, 
    (uint8_t)(fgrPrint->capacity >> 8),
    (uint8_t)(fgrPrint->capacity & 0xFF)
  };

  // search of slot starting thru the capacity
  status = sendCommand(fgrPrint, FINGERPRINT_SEARCH, data, sizeof(data));
  if (status != FINGERPRINT_OK) return status;

  fgrPrint->fingerID = 0xFFFF;
  fgrPrint->confidence = 0xFFFF;

  fgrPrint->fingerID = packet->data[1];
  fgrPrint->fingerID <<= 8;
  fgrPrint->fingerID |= packet->data[2];

  fgrPrint->confidence = packet->data[3];
  fgrPrint->confidence <<= 8;
  fgrPrint->confidence |= packet->data[4];

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
uint8_t AFGR_GetTemplateCount(Fingerprint_t *fgrPrint)
{
  uint8_t status;
  Fingerprint_Packet_t *packet = &(fgrPrint)->packet;

  status = sendCommand(fgrPrint, FINGERPRINT_TEMPLATECOUNT, 0, 9);
  if (status != FINGERPRINT_OK) return status;

  fgrPrint->templateCount = packet->data[1];
  fgrPrint->templateCount <<= 8;
  fgrPrint->templateCount |= packet->data[2];

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
uint8_t AFGR_SetPassword(Fingerprint_t *fgrPrint, uint32_t password)
{
  uint32_t bigEndianPassword = uint32ToBigEndian(password);

  return sendCommand(fgrPrint, FINGERPRINT_SETPASSWORD, &bigEndianPassword, sizeof(uint32_t));
}


/**************************************************************************/
/*!
    @brief   Control the built in LED
    @param on True if you want LED on, False to turn LED off
    @returns <code>FINGERPRINT_OK</code> on success
*/
/**************************************************************************/
uint8_t AFGR_LEDcontrol(Fingerprint_t *fgrPrint, uint8_t on)
{
  if (on) {
    return sendCommand(fgrPrint, FINGERPRINT_LEDON, 0, 0);
  } else {
    return sendCommand(fgrPrint, FINGERPRINT_LEDOFF, 0, 0);
  }
}


static uint8_t sendCommand(Fingerprint_t *fgrPrint, uint8_t command, uint8_t *data, uint16_t length)
{
  Fingerprint_Packet_t *packet = &(fgrPrint)->packet;
  uint8_t status = FINGERPRINT_PACKETRECIEVEERR;

  if (fgrPrint->mutexLock(2000) != 0) {
    status = FINGERPRINT_TIMEOUT;
    goto end;
  }

  if (length > FINGERPRINT_PACKET_DATA_MAX_SZ) {
    // return error
  }

  packet->type = FINGERPRINT_COMMANDPACKET;
  packet->command = command;
  packet->datalength = length;
  packet->data = data;
  sendPacket(fgrPrint, packet);
  if (getResponse(fgrPrint, packet, 5000) != FINGERPRINT_OK)
    goto end;
  if (packet->type != FINGERPRINT_ACKPACKET)
    goto end;
  status = packet->data[0];
end:
  fgrPrint->mutexUnlock();
  return status;
}


static void sendPacket(Fingerprint_t *fgrPrint, const Fingerprint_Packet_t *packet)
{
  uint16_t i = 0, j;
  uint16_t checksum = 0;
  uint8_t *buffer = fgrPrint->txBuffer;

  if (buffer == 0) return;
  if (fgrPrint->txBufferSize < 9) return;
  if (fgrPrint->write == 0) return;

  *(buffer + (i++)) = (uint8_t)((AFGR_START_CODE >> 8) & 0xFF);
  *(buffer + (i++)) = (uint8_t)(AFGR_START_CODE & 0xFF);
  *(buffer + (i++)) = (uint8_t)((fgrPrint->address >> 24) & 0xFF);
  *(buffer + (i++)) = (uint8_t)((fgrPrint->address >> 16) & 0xFF);
  *(buffer + (i++)) = (uint8_t)((fgrPrint->address >> 8) & 0xFF);
  *(buffer + (i++)) = (uint8_t)((fgrPrint->address) & 0xFF);
  *(buffer + (i++)) = packet->type;
  checksum = packet->type;

  uint16_t wire_length = packet->datalength + 2; // 2 bytes checksum
  if (packet->type == FINGERPRINT_COMMANDPACKET) wire_length++;
  *(buffer + (i++)) = (uint8_t)((wire_length >> 8)&0xFF);
  *(buffer + (i++)) = (uint8_t)(wire_length&0xFF);

  checksum += (((wire_length >> 8)&0xFF) + (wire_length&0xFF));

  if (packet->type == FINGERPRINT_COMMANDPACKET)
  {
    *(buffer + (i++)) = packet->command;
    checksum += packet->command;
  }

  for (j = 0; j < packet->datalength; j++) {
    if (i >= fgrPrint->txBufferSize) {
      fgrPrint->write(buffer, i);
      i = 0;
    }
    *(buffer + (i++)) = *(packet->data + j);
    checksum += *(packet->data + j);
  }

  if (i >= (fgrPrint->txBufferSize-2)) {
    fgrPrint->write(buffer, i);
    i = 0;
  }

  *(buffer + (i++)) = (uint8_t)(checksum >> 8);
  *(buffer + (i++)) = (uint8_t)(checksum & 0xFF);
  fgrPrint->write(buffer, i);
  return;
}

static uint8_t getResponse(Fingerprint_t *fgrPrint, Fingerprint_Packet_t *packet, uint32_t timeout)
{
  uint8_t byte;
  int16_t idx = 0;
  uint16_t start_code = 0;
  int readResult;
  uint8_t *buffer = fgrPrint->rxBuffer;
  uint32_t address = 0;
  uint16_t checksum;
  uint16_t recvChecksum;

  if (fgrPrint->read == 0) return FINGERPRINT_ERROR;
  if (fgrPrint->getTick == 0) return FINGERPRINT_ERROR;
  if (fgrPrint->rxBuffer == 0) return FINGERPRINT_ERROR;

  while (1) {
    readResult = fgrPrint->read(&byte, 1, timeout);
    if (readResult <= 0) return FINGERPRINT_TIMEOUT;

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
      address |= (uint32_t)byte << (8*(5-idx));
      break;
    case 6:
      packet->type = byte;
      checksum = byte;
      break;
    case 7:
      packet->datalength = (uint16_t)byte << 8;
      checksum += byte;
      break;
    case 8:
      packet->datalength |= byte;
      if (packet->datalength > fgrPrint->rxBufferSize)
        return FINGERPRINT_BADPACKET;
      checksum += byte;

      // get data;
      break;

    // checksum
    case -2:
      recvChecksum = (uint16_t)byte << 8;
      break;
    case -1:
      recvChecksum |= (uint16_t)byte;
      if (recvChecksum != checksum) {
        return FINGERPRINT_BADPACKET;
      }
      return FINGERPRINT_OK;
      break;
    default:
      *buffer = byte;
      buffer++;
      checksum += byte;
      if ((idx - 6) >= packet->datalength) {
        packet->data = fgrPrint->rxBuffer;
        idx = -3;
      }
      break;
    }
    idx++;
  }
  // Shouldn't get here so...
  return FINGERPRINT_BADPACKET;

}


static uint16_t uint16ToBigEndian(uint16_t num)
{
  uint16_t result;
  uint8_t *bytesResult = &result;

  *bytesResult      = (uint8_t)((num >> 8) & 0xFF);
  *(bytesResult+1)  = (uint8_t)(num & 0xFF);

  return result;
}

static uint32_t uint32ToBigEndian(uint32_t num)
{
  uint32_t result;
  uint8_t *bytesResult = &result;

  *bytesResult      = (uint8_t)((num >> 24) & 0xFF);
  *(bytesResult+1)  = (uint8_t)((num >> 16) & 0xFF);
  *(bytesResult+2)  = (uint8_t)((num >> 8) & 0xFF);
  *(bytesResult+3)  = (uint8_t)(num & 0xFF);

  return result;
}

static uint8_t mutexLock(uint32_t timeout)
{
  return FINGERPRINT_OK;
}

static uint8_t mutexUnlock(void)
{
  return FINGERPRINT_OK;
}
