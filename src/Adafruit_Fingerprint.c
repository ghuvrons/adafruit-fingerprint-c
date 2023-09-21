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
static Fingerprint_Error_t sendCommand(Fingerprint_t*, uint8_t command, 
                                       uint8_t *params, uint16_t paramsLength,
                                       uint8_t *respBuffer, uint16_t respBufferSize);
static Fingerprint_Error_t sendCommandWithData(Fingerprint_t*, uint8_t command, 
                                               const uint8_t *params, uint16_t paramsLength, 
                                               uint8_t *data, uint16_t dataLength);
static int16_t sendCommandWithResponseData(Fingerprint_t*, uint8_t command, 
                                           uint8_t *params, uint16_t paramsLength, 
                                           uint8_t *respBuffer, uint16_t respBufferSize);
static Fingerprint_Error_t sendPacket(Fingerprint_t*, Fingerprint_Packet_t*);
static Fingerprint_Error_t getResponse(Fingerprint_t*, Fingerprint_Packet_t*, 
                                       uint8_t *buffer, uint16_t bufferSize, 
                                       uint32_t timeout);
static uint16_t uint16ToBigEndian(uint16_t);
static uint32_t uint32ToBigEndian(uint32_t);
static uint16_t bigEndianToUint16(uint16_t);
static uint32_t bigEndianToUint32(uint32_t);

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

uint8_t AFGR_Init(Fingerprint_t *fgrPrint,
                  uint8_t *txBuffer, uint16_t txBufferSize,
                  uint8_t *rxBuffer, uint16_t rxBufferSize)
{
  if (fgrPrint->delay == 0
      || fgrPrint->getTick == 0
      || fgrPrint->serial.read == 0
      || fgrPrint->serial.write == 0
      || fgrPrint->serial.flush == 0)
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

/**
 * @brief Set the password on the sensor (future communication will require
 * password verification so don't forget it!!!)
 * @param   password 32-bit password code
 * @returns Fingerprint_Error_t
*/
Fingerprint_Error_t AFGR_SetPassword(Fingerprint_t *fgrPrint, uint32_t password)
{
  uint32_t bigEndianPassword = uint32ToBigEndian(password);
  return sendCommand(fgrPrint, FINGERPRINT_SETPASSWORD, (uint8_t*) &bigEndianPassword, sizeof(uint32_t), 0, 0);
}

/**
 * @brief Verifies the sensors' access password (default password is 0x0000000). 
 * A good way to also check if the sensors is active and responding
*/
Fingerprint_Error_t AFGR_VerifyPassword(Fingerprint_t *fgrPrint)
{
  uint32_t bigEndianPassword = uint32ToBigEndian(fgrPrint->password);
  return sendCommand(fgrPrint, FINGERPRINT_VERIFYPASSWORD, 
                     (uint8_t*) &bigEndianPassword, sizeof(uint32_t),
                     0, 0);
}


/**************************************************************************/
/*!
    @brief  Get the sensors parameters, fills in the member variables
    status_reg, system_id, capacity, security_level, device_addr, packet_len
    and baud_rate
    @returns True if password is correct
*/
/**************************************************************************/
Fingerprint_Error_t AFGR_GetParameters(Fingerprint_t *fgrPrint)
{
  Fingerprint_Error_t status;

  struct __attribute__((packed)){
    uint16_t status_reg;
    uint16_t system_id;
    uint16_t capacity;
    uint16_t security_level;
    uint32_t device_addr;
    uint16_t packet_len;
    uint16_t baud_rate;
  } data;


  status = sendCommand(fgrPrint, FINGERPRINT_READSYSPARAM, 0, 0, (uint8_t*) &data, sizeof(data));
  if (status != FINGERPRINT_OK) return status;

   fgrPrint->status_reg      = bigEndianToUint16(data.status_reg);
   fgrPrint->system_id       = bigEndianToUint16(data.system_id);
   fgrPrint->capacity        = bigEndianToUint16(data.capacity);
   fgrPrint->security_level  = bigEndianToUint16(data.security_level);
   fgrPrint->device_addr     = bigEndianToUint32(data.device_addr);
   fgrPrint->packet_len      = bigEndianToUint16(data.packet_len);
   fgrPrint->baud_rate       = bigEndianToUint16(data.baud_rate) * 9600;

   if (fgrPrint->packet_len == 0) {
     fgrPrint->packet_len = 32;
   } else if (fgrPrint->packet_len == 1) {
     fgrPrint->packet_len = 64;
   } else if (fgrPrint->packet_len == 2) {
     fgrPrint->packet_len = 128;
   } else if (fgrPrint->packet_len == 3) {
     fgrPrint->packet_len = 256;
   }

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
Fingerprint_Error_t AFGR_GetImage(Fingerprint_t *fgrPrint)
{
  return sendCommand(fgrPrint, FINGERPRINT_GETIMAGE, 0, 0, 0, 0);
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
Fingerprint_Error_t AFGR_Image2Tz(Fingerprint_t *fgrPrint, Fingerprint_CharBuffer_t charBuffer)
{
  return sendCommand(fgrPrint, FINGERPRINT_IMAGE2TZ, (uint8_t*)&charBuffer, 1, 0, 0);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to take two print feature template and create a
   model
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_ENROLLMISMATCH</code> on mismatch of fingerprints
*/
Fingerprint_Error_t AFGR_CreateModel(Fingerprint_t *fgrPrint)
{
  return sendCommand(fgrPrint, FINGERPRINT_REGMODEL, 0, 0, 0, 0);
}


Fingerprint_Error_t AFGR_CreateModelFromBuffer(Fingerprint_t *fgrPrint, uint8_t *data, uint16_t size)
{
  Fingerprint_Error_t status;
  uint8_t charBuffer = (uint8_t) FINGERPRINT_CHAR_BUFFER_1;

  status = sendCommandWithData(fgrPrint, FINGERPRINT_DOWNLOAD, &charBuffer, 1, data, size);
  if (status != FINGERPRINT_OK) return status;

  return status;
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
Fingerprint_Error_t AFGR_StoreModel(Fingerprint_t *fgrPrint, uint16_t id)
{
  uint8_t data[] = {
    (uint8_t)FINGERPRINT_CHAR_BUFFER_1,
    (uint8_t)(id >> 8),
    (uint8_t)(id & 0xFF)
  };

  return sendCommand(fgrPrint, FINGERPRINT_STORE, data, sizeof(data), 0, 0);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to load a fingerprint model from flash into buffer 1
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
Fingerprint_Error_t AFGR_LoadModel(Fingerprint_t *fgrPrint, uint16_t id)
{
  uint8_t data[] = {
    (uint8_t)FINGERPRINT_CHAR_BUFFER_1,
    (uint8_t)(id >> 8),
    (uint8_t)(id & 0xFF)
  };

  return sendCommand(fgrPrint, FINGERPRINT_LOAD, data, sizeof(data), 0, 0);
}

/**
 * @brief   Ask the sensor to transfer 256-byte fingerprint template from the
 * buffer to the UART
 * @returns length of get data
 * @returns return Fingerprint error if < 0. error can be get from return value & 0x08
*/
int16_t AFGR_GetModel(Fingerprint_t *fgrPrint, uint8_t *buffer, uint16_t size)
{
  uint8_t charBuffer = (uint8_t) FINGERPRINT_CHAR_BUFFER_1;
  return sendCommandWithResponseData(fgrPrint, FINGERPRINT_UPLOAD, &charBuffer, 1, buffer, size);
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
Fingerprint_Error_t AFGR_DeleteModel(Fingerprint_t *fgrPrint, uint16_t id)
{
  uint8_t data[] = {
    (uint8_t)(id >> 8),
    (uint8_t)(id & 0xFF), 
    0x00, 
    0x01
  };

  return sendCommand(fgrPrint, FINGERPRINT_DELETE, data, sizeof(data), 0, 0);
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
Fingerprint_Error_t AFGR_EmptyDatabase(Fingerprint_t *fgrPrint)
{
  return sendCommand(fgrPrint, FINGERPRINT_EMPTY, 0, 0, 0, 0);
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
Fingerprint_Error_t AFGR_FingerFastSearch(Fingerprint_t *fgrPrint, Fingerprint_CharBuffer_t charBuffer)
{
  Fingerprint_Error_t status;
  uint8_t data[] = {
    (uint8_t)charBuffer, 0x00, 0x00, 0x00, 0xA3
  };

  struct __attribute__((packed))
  {
    uint16_t fingerID;
    uint16_t confidence;
  } resp;

  // high speed search of slot #1 starting at page 0x0000 and page #0x00A3
  status = sendCommand(fgrPrint, FINGERPRINT_HISPEEDSEARCH, data, sizeof(data), (uint8_t*) &resp, sizeof(resp));
  if (status != FINGERPRINT_OK) return status;

  fgrPrint->fingerID = bigEndianToUint16(resp.fingerID);
  fgrPrint->confidence = bigEndianToUint16(resp.confidence);

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
Fingerprint_Error_t AFGR_FingerSearch(Fingerprint_t *fgrPrint, Fingerprint_CharBuffer_t charBuffer)
{
  Fingerprint_Error_t status;
  uint8_t data[] = {
    (uint8_t)charBuffer, 
    0x00, 
    0x00, 
    (uint8_t)(fgrPrint->capacity >> 8),
    (uint8_t)(fgrPrint->capacity & 0xFF)
  };

  struct __attribute__((packed))
  {
    uint16_t fingerID;
    uint16_t confidence;
  } resp;
  
  // search of slot starting thru the capacity
  status = sendCommand(fgrPrint, FINGERPRINT_SEARCH, data, sizeof(data), (uint8_t*) &resp, sizeof(resp));
  if (status != FINGERPRINT_OK) return status;

  fgrPrint->fingerID = bigEndianToUint16(resp.fingerID);
  fgrPrint->confidence = bigEndianToUint16(resp.confidence);

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
Fingerprint_Error_t AFGR_GetTemplateCount(Fingerprint_t *fgrPrint)
{
  Fingerprint_Error_t status;
  uint16_t templateCount;

  status = sendCommand(fgrPrint, FINGERPRINT_TEMPLATECOUNT, 0, 0, (uint8_t*) &templateCount, sizeof(uint16_t));
  if (status != FINGERPRINT_OK) return status;

  fgrPrint->templateCount = bigEndianToUint16(templateCount);

  return status;
}


/**************************************************************************/
/*!
    @brief   Control the built in LED
    @param on True if you want LED on, False to turn LED off
    @returns <code>FINGERPRINT_OK</code> on success
*/
/**************************************************************************/
Fingerprint_Error_t AFGR_LEDcontrol(Fingerprint_t *fgrPrint, uint8_t on)
{
  if (on) {
    return sendCommand(fgrPrint, FINGERPRINT_LEDON, 0, 0, 0, 0);
  } else {
    return sendCommand(fgrPrint, FINGERPRINT_LEDOFF, 0, 0, 0, 0);
  }
}


static Fingerprint_Error_t sendCommand(Fingerprint_t *fgrPrint, uint8_t command, 
                                       uint8_t *params, uint16_t paramsLength, 
                                       uint8_t *respBuffer, uint16_t respBufferSize)
{
  Fingerprint_Packet_t *packet = &(fgrPrint)->packet;
  Fingerprint_Error_t status = FINGERPRINT_PACKETRECIEVEERR;

  if (fgrPrint->mutexLock(2000) != 0) {
    return FINGERPRINT_TIMEOUT;
  }

  if (fgrPrint->serial.flush == 0) return FINGERPRINT_ERROR;
  fgrPrint->serial.flush();

  packet->header.type = FINGERPRINT_COMMANDPACKET;
  packet->command = command;
  packet->datalength = paramsLength;
  packet->data = params;
  status = sendPacket(fgrPrint, packet);
  if (status != FINGERPRINT_OK) goto end;

  status = getResponse(fgrPrint, packet, respBuffer, respBufferSize, 1000);
  if (status != FINGERPRINT_OK) goto end;
  if (packet->header.type != FINGERPRINT_ACKPACKET) goto end;
  if (packet->response != FINGERPRINT_OK) {
    status = packet->response;
    goto end;
  } else {
    status = FINGERPRINT_OK;
  }

end:
  fgrPrint->mutexUnlock();
  return status;
}


static Fingerprint_Error_t sendCommandWithData(Fingerprint_t *fgrPrint, uint8_t command, 
                                               const uint8_t *params, uint16_t paramsLength, 
                                               uint8_t *data, uint16_t dataLength)
{
  Fingerprint_Packet_t *packet = &(fgrPrint)->packet;
  Fingerprint_Error_t status = FINGERPRINT_PACKETRECIEVEERR;

  if (fgrPrint->packet_len == 0)   return FINGERPRINT_ERROR;
  if (fgrPrint->serial.flush == 0) return FINGERPRINT_ERROR;

  if (fgrPrint->mutexLock(2000) != 0) {
    return FINGERPRINT_TIMEOUT;
  }

  fgrPrint->serial.flush();

  packet->header.type = FINGERPRINT_COMMANDPACKET;
  packet->command = command;
  packet->datalength = paramsLength;
  packet->data = params;
  status = sendPacket(fgrPrint, packet);
  if (status != FINGERPRINT_OK) goto end;

  status = getResponse(fgrPrint, packet, 0, 0, 1000);
  if (status != FINGERPRINT_OK) goto end;
  if (packet->header.type != FINGERPRINT_ACKPACKET) goto end;
  if (packet->response != FINGERPRINT_OK) {
    status = packet->response;
    goto end;
  } else {
    status = FINGERPRINT_OK;
  }
  
  while (dataLength > 0) {
    packet->header.type = FINGERPRINT_DATAPACKET;
    packet->datalength = dataLength;
    packet->data = data;

    if (packet->datalength > fgrPrint->packet_len) {
      packet->datalength = fgrPrint->packet_len;
    }
    else {
      packet->header.type = FINGERPRINT_ENDDATAPACKET;
    }

    status = sendPacket(fgrPrint, packet);
    if (status != FINGERPRINT_OK) goto end;

    dataLength -= packet->datalength;
    data += packet->datalength;
  }

end:
  fgrPrint->mutexUnlock();
  return status;
}


static int16_t sendCommandWithResponseData(Fingerprint_t *fgrPrint, uint8_t command, 
                                           uint8_t *params, uint16_t paramsLength, 
                                           uint8_t *respBuffer, uint16_t respBufferSize)
{
  Fingerprint_Packet_t *packet = &(fgrPrint)->packet;
  Fingerprint_Error_t status = FINGERPRINT_PACKETRECIEVEERR;
  uint16_t respDataLength = 0;

  if (fgrPrint->mutexLock(2000) != 0) {
    return FINGERPRINT_TIMEOUT;
  }

  if (fgrPrint->serial.flush == 0) return FINGERPRINT_ERROR;
  fgrPrint->serial.flush();

  packet->header.type = FINGERPRINT_COMMANDPACKET;
  packet->command = command;
  packet->datalength = paramsLength;
  packet->data = params;
  status = sendPacket(fgrPrint, packet);
  if (status != FINGERPRINT_OK) goto end;

  status = getResponse(fgrPrint, packet, respBuffer, respBufferSize, 1000);
  if (status != FINGERPRINT_OK) goto end;
  if (packet->header.type != FINGERPRINT_ACKPACKET) goto end;
  if (packet->response != FINGERPRINT_OK) {
    status = packet->response;
    goto end;
  } else {
    status = FINGERPRINT_OK;
  }

getResponseData:
  status = getResponse(fgrPrint, packet, 
                      (respBuffer)?respBuffer+respDataLength:0, respBufferSize, 
                      1000);
  if (status != FINGERPRINT_OK && status != FINGERPRINT_BUF_SIZE_TOO_SHORT) 
    goto end;
  if (respBuffer && respBufferSize > 0) {
    respBufferSize -= packet->datalength;
    respDataLength += packet->datalength;
  }
  if (packet->header.type != FINGERPRINT_ENDDATAPACKET)
    goto getResponseData;

end:
  fgrPrint->mutexUnlock();
  if (status != FINGERPRINT_OK) {
    return ((int16_t) status)|0x8000;
  }

  if (respBuffer && respDataLength != 0) {
    return (int16_t) respDataLength;
  }

  return (int16_t) status;
}


static Fingerprint_Error_t sendPacket(Fingerprint_t *fgrPrint, Fingerprint_Packet_t *packet)
{
  uint8_t *buffer = fgrPrint->txBuffer;
  uint16_t bufferSize = fgrPrint->txBufferSize;
  uint16_t bufferLength = 0;

  if (buffer == 0) return FINGERPRINT_ERROR;
  if (fgrPrint->txBufferSize < 9) return FINGERPRINT_ERROR;
  if (fgrPrint->serial.write == 0) return FINGERPRINT_ERROR;

  packet->header.prefix = uint16ToBigEndian(AFGR_START_CODE);
  packet->header.address = uint32ToBigEndian(fgrPrint->address);
  packet->header.payloadlength = packet->datalength + 2;

  if (packet->header.type == FINGERPRINT_COMMANDPACKET)
  {
    packet->header.payloadlength++;
  }
  packet->checksum = packet->header.type;
  packet->checksum += ((packet->header.payloadlength >> 8) & 0xFF);
  packet->checksum += packet->header.payloadlength & 0xFF;
  packet->header.payloadlength = uint16ToBigEndian(packet->header.payloadlength);

  // write header
  memcpy(buffer, &packet->header, sizeof(Fingerprint_PacketHeader_t));
  buffer += sizeof(Fingerprint_PacketHeader_t);
  bufferSize -= sizeof(Fingerprint_PacketHeader_t);
  bufferLength += sizeof(Fingerprint_PacketHeader_t);

  // write command
  if (packet->header.type == FINGERPRINT_COMMANDPACKET)
  {
    *buffer = packet->command;
    buffer++;
    bufferSize--;
    bufferLength++;
    packet->checksum += packet->command;
  }

  // write data parameter
  for (uint16_t j = 0; j < packet->datalength; j++) {
    if (bufferSize == 0) {
      fgrPrint->serial.write(fgrPrint->txBuffer, bufferLength);
      buffer = fgrPrint->txBuffer;
      bufferSize = fgrPrint->txBufferSize;
      bufferLength = 0;
    }

    packet->checksum += *(packet->data + j);
    *buffer = *(packet->data + j);
    buffer++;
    bufferSize--;
    bufferLength++;
  }

  if (bufferSize < 2) {
    fgrPrint->serial.write(fgrPrint->txBuffer, bufferLength);
    buffer = fgrPrint->txBuffer;
    bufferSize = fgrPrint->txBufferSize;
    bufferLength = 0;
  }

  packet->checksum = uint16ToBigEndian(packet->checksum);
  memcpy(buffer, &packet->checksum, sizeof(uint16_t));
  bufferSize += sizeof(uint16_t);
  bufferLength += sizeof(uint16_t);

  fgrPrint->serial.write(fgrPrint->txBuffer, bufferLength);
  return FINGERPRINT_OK;
}

static Fingerprint_Error_t getResponse(Fingerprint_t *fgrPrint, 
                                       Fingerprint_Packet_t *packet, 
                                       uint8_t *buffer, uint16_t bufferSize, 
                                       uint32_t timeout)
{
  uint8_t byte;
  int16_t idx = 0;
  int readResult;
  uint16_t checksum;
  Fingerprint_Error_t err = FINGERPRINT_OK;

  if (fgrPrint->serial.read == 0) return FINGERPRINT_ERROR;
  if (fgrPrint->getTick == 0) return FINGERPRINT_ERROR;
  if (fgrPrint->rxBuffer == 0) return FINGERPRINT_ERROR;
  
  // read prefix
  while (idx < 2) {
    readResult = fgrPrint->serial.read(&byte, 1, timeout);
    if (readResult <= 0) return FINGERPRINT_TIMEOUT;

    switch (idx) {
    case 0:
      if (byte != (AFGR_START_CODE >> 8))
        continue;
      packet->header.prefix = (uint16_t)byte << 8;
      break;

    case 1:
      packet->header.prefix |= byte;
      if (packet->header.prefix != AFGR_START_CODE)
        return FINGERPRINT_BADPACKET;
      break;
    }
    idx++;
  }

  // read header 
  readResult = fgrPrint->serial.read((uint8_t*)&packet->header.address, sizeof(Fingerprint_PacketHeader_t)-2, timeout);
  if (readResult <= 0) return FINGERPRINT_TIMEOUT;

  packet->header.address = bigEndianToUint32(packet->header.address);
  packet->header.payloadlength = bigEndianToUint16(packet->header.payloadlength);
  if (packet->header.payloadlength == 0) return FINGERPRINT_BADPACKET;

  checksum = packet->header.type;
  checksum += (packet->header.payloadlength >> 8) & 0xFFU;
  checksum += (packet->header.payloadlength) & 0xFFU;

  // read response code
  if (packet->header.type == FINGERPRINT_ACKPACKET
      && packet->header.payloadlength > 1)
  {
    readResult = fgrPrint->serial.read(&packet->response, sizeof(uint8_t), timeout);
    if (readResult <= 0) return FINGERPRINT_TIMEOUT;
    checksum += packet->response;
    packet->header.payloadlength--;
  }

  if (packet->header.payloadlength < 2) return FINGERPRINT_BADPACKET;

  // read response data
  packet->data = buffer;
  packet->datalength = 0;
  packet->header.payloadlength -= 2; // not read checksum

  while (buffer && bufferSize && packet->header.payloadlength) {
    readResult = fgrPrint->serial.read(buffer, 
                                bufferSize < packet->header.payloadlength? 
                                  bufferSize:
                                  packet->header.payloadlength, 
                                timeout);
    
    if (readResult < 0) 
      return FINGERPRINT_TIMEOUT;

    packet->header.payloadlength -= readResult;
    packet->datalength += readResult;
    for (uint16_t i = 0; i < readResult; i++) 
    {
      checksum += *buffer;
      buffer++;
      bufferSize--;
    }
  }

  while (packet->header.payloadlength)
  {
    err = FINGERPRINT_BUF_SIZE_TOO_SHORT;
    readResult = fgrPrint->serial.read(&byte, sizeof(uint8_t), timeout);
    if (readResult <= 0)
      return FINGERPRINT_TIMEOUT;
      
    checksum += byte;
    packet->header.payloadlength--;
  }

  // checksum
  readResult = fgrPrint->serial.read((uint8_t*)&packet->checksum, sizeof(uint16_t), timeout);
  if (readResult <= 0) return FINGERPRINT_TIMEOUT;

  packet->checksum = bigEndianToUint16(packet->checksum);
  if (checksum != packet->checksum) return FINGERPRINT_BADPACKET;

  return err;
}


static uint16_t uint16ToBigEndian(uint16_t num)
{
  uint16_t result;
  uint8_t *bytesResult = (uint8_t*) &result;

  *bytesResult      = (uint8_t)((num >> 8) & 0xFF);
  *(bytesResult+1)  = (uint8_t)(num & 0xFF);

  return result;
}

static uint32_t uint32ToBigEndian(uint32_t num)
{
  uint32_t result;
  uint8_t *bytesResult = (uint8_t*) &result;

  *bytesResult      = (uint8_t)((num >> 24) & 0xFF);
  *(bytesResult+1)  = (uint8_t)((num >> 16) & 0xFF);
  *(bytesResult+2)  = (uint8_t)((num >> 8) & 0xFF);
  *(bytesResult+3)  = (uint8_t)(num & 0xFF);

  return result;
}

static uint16_t bigEndianToUint16(uint16_t bigendianNum)
{
  uint16_t result;
  uint8_t *bigendian = (uint8_t*) &bigendianNum;

  result = ((uint16_t) *bigendian) << 8;
  result |= ((uint16_t) *(bigendian+1));

  return result;
}


static uint32_t bigEndianToUint32(uint32_t bigendianNum)
{
  uint32_t result;
  uint8_t *bigendian = (uint8_t*) &bigendianNum;

  result = ((uint32_t) *bigendian) << 24;
  result |= ((uint32_t) *(bigendian+1)) << 16;
  result |= ((uint32_t) *(bigendian+2)) << 8;
  result |= ((uint32_t) *(bigendian+3));

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
