#ifndef ADAFRUIT_FINGERPRINT_CLIB_H
#define ADAFRUIT_FINGERPRINT_CLIB_H

#include "Adafruit_Fingerprint/conf.h"
#include <stdint.h>

/*!
 * @file Adafruit_Fingerprint.h
 */


#define FINGERPRINT_STARTCODE          0xEF01 //!< Fixed falue of EF01H; High byte transferred first

#define FINGERPRINT_COMMANDPACKET      0x1  //!< Command packet
#define FINGERPRINT_DATAPACKET         0x2  //!< Data packet, must follow command packet
                                            //!  or acknowledge packet
#define FINGERPRINT_ACKPACKET          0x7  //!< Acknowledge packet
#define FINGERPRINT_ENDDATAPACKET      0x8  //!< End of data packet

#define FINGERPRINT_GETIMAGE           0x01 //!< Collect finger image
#define FINGERPRINT_IMAGE2TZ           0x02 //!< Generate character file from image
#define FINGERPRINT_SEARCH             0x04 //!< Search for fingerprint in slot
#define FINGERPRINT_REGMODEL           0x05 //!< Combine character files and generate template
#define FINGERPRINT_STORE              0x06 //!< Store template
#define FINGERPRINT_LOAD               0x07 //!< Read/load template
#define FINGERPRINT_UPLOAD             0x08 //!< Upload template
#define FINGERPRINT_DOWNLOAD           0x09 //!< Download template
#define FINGERPRINT_DELETE             0x0C //!< Delete templates
#define FINGERPRINT_EMPTY              0x0D //!< Empty library
#define FINGERPRINT_READSYSPARAM       0x0F //!< Read system parameters
#define FINGERPRINT_SETPASSWORD        0x12 //!< Sets passwords
#define FINGERPRINT_VERIFYPASSWORD     0x13 //!< Verifies the password
#define FINGERPRINT_HISPEEDSEARCH      0x1B //!< Asks the sensor to search
                                            //!  for a matching fingerprint template to the
                                            //!  last model generated
#define FINGERPRINT_TEMPLATECOUNT      0x1D //!< Read finger template numbers
#define FINGERPRINT_AURALEDCONFIG      0x35 //!< Aura LED control
#define FINGERPRINT_LEDON              0x50 //!< Turn on the onboard LED
#define FINGERPRINT_LEDOFF             0x51 //!< Turn off the onboard LED

#define FINGERPRINT_LED_BREATHING      0x01 //!< Breathing light
#define FINGERPRINT_LED_FLASHING       0x02 //!< Flashing light
#define FINGERPRINT_LED_ON             0x03 //!< Always on
#define FINGERPRINT_LED_OFF            0x04 //!< Always off
#define FINGERPRINT_LED_GRADUAL_ON     0x05 //!< Gradually on
#define FINGERPRINT_LED_GRADUAL_OFF    0x06 //!< Gradually off
#define FINGERPRINT_LED_RED            0x01 //!< Red LED
#define FINGERPRINT_LED_BLUE           0x02 //!< Blue LED
#define FINGERPRINT_LED_PURPLE         0x03 //!< Purple LED

//#define FINGERPRINT_DEBUG

#define DEFAULTTIMEOUT 1000 //!< UART reading timeout in milliseconds

typedef enum {
  FINGERPRINT_OK                 = 0,
  FINGERPRINT_ERROR              = -1, //!< Command execution is error
  FINGERPRINT_TIMEOUT            = -2, //!< Timeout was reached
  FINGERPRINT_BADPACKET          = -3, //!< Bad packet was sent

  FINGERPRINT_PACKETRECIEVEERR   = 0x01, //!< Error when receiving data package
  FINGERPRINT_NOFINGER           = 0x02, //!< No finger on the sensor
  FINGERPRINT_IMAGEFAIL          = 0x03, //!< Failed to enroll the finger
  FINGERPRINT_IMAGEMESS          = 0x06, //!< Failed to generate character file due
                                            //!  to overly disorderly fingerprint image
  FINGERPRINT_FEATUREFAIL        = 0x07, //!< Failed to generate character file due
                                            //!  to the lack of character point
                                            //!  or small fingerprint image
  FINGERPRINT_NOMATCH            = 0x08, //!< Finger doesn't match
  FINGERPRINT_NOTFOUND           = 0x09, //!< Failed to find matching finger
  FINGERPRINT_ENROLLMISMATCH     = 0x0A, //!< Failed to combine the character files
  FINGERPRINT_BADLOCATION        = 0x0B, //!< Addressed PageID is beyond the finger library
  FINGERPRINT_DBRANGEFAIL        = 0x0C, //!< Error when reading template from library
                                            //!  or invalid template
  FINGERPRINT_UPLOADFEATUREFAIL  = 0x0D, //!< Error when uploading template
  FINGERPRINT_PACKETRESPONSEFAIL = 0x0E, //!< Module failed to receive the following data packages
  FINGERPRINT_UPLOADFAIL         = 0x0F, //!< Error when uploading image
  FINGERPRINT_DELETEFAIL         = 0x10, //!< Failed to delete the template
  FINGERPRINT_DBCLEARFAIL        = 0x11, //!< Failed to clear finger library
  FINGERPRINT_PASSFAIL           = 0x13, //!< Find whether the fingerprint passed or failed
  FINGERPRINT_INVALIDIMAGE       = 0x15, //!< Failed to generate image because of lac
                                            //!  of valid primary image
  FINGERPRINT_FLASHERR           = 0x18, //!< Error when writing flash
  FINGERPRINT_INVALIDREG         = 0x1A, //!< Invalid register number
  FINGERPRINT_ADDRCODE           = 0x20, //!< Address code
  FINGERPRINT_PASSVERIFY         = 0x21, //!< Verify the fingerprint passed
} Fingerprint_Error_t;

typedef enum {
  FINGERPRINT_CHAR_BUFFER_1,
  FINGERPRINT_CHAR_BUFFER_2,
} Fingerprint_CharBuffer_t;

typedef struct __attribute__((packed)) {
  uint16_t prefix;
  uint32_t address;
  uint8_t  type;          ///< Type of packet
  uint16_t payloadlength; ///< Length of packet
} Fingerprint_PacketHeader_t;

///! Helper class to craft UART packets
typedef struct  {

  /**************************************************************************/
  /*!
      @brief   Create a new UART-borne packet
      @param   type Command, data, ack type packet
      @param   length Size of payload
      @param   data Pointer to bytes of size length we will memcopy into the
     internal buffer
  */
  /**************************************************************************/
  Fingerprint_PacketHeader_t header;

  // payload
  uint8_t  command;
  uint8_t  response;
  const uint8_t *data;        ///< The raw buffer for packet payload
  uint16_t datalength;
  uint16_t checksum;
} Fingerprint_Packet_t;

typedef struct {

  uint32_t thePassword;
  uint32_t address;

  /// The matching location that is set by fingerFastSearch()
  uint16_t fingerID;
  /// The confidence of the fingerFastSearch() match, higher numbers are more
  /// confidents
  uint16_t confidence;
  /// The number of stored templates in the sensor, set by getTemplateCount()
  uint16_t templateCount;

  uint16_t status_reg; ///< The status register (set by getParameters)
  uint16_t system_id;  ///< The system identifier (set by getParameters)
  uint16_t capacity; ///< The fingerprint capacity (set by getParameters)
  uint16_t security_level; ///< The security level (set by getParameters)
  uint32_t device_addr;             ///< The device address (set by getParameters)
  uint16_t packet_len;   ///< The max packet length (set by getParameters)
  uint16_t baud_rate; ///< The UART baud rate (set by getParameters)

  void      (*delay)(uint32_t ms);
  uint32_t  (*getTick)(void);
  uint8_t   (*mutexLock)(uint32_t timeout);
  uint8_t   (*mutexUnlock)(void);
  int       (*write)(uint8_t *data, uint16_t length);

  /**
   * @brief 
   * @result read length, -1 if error timeout, 0 if EOF
   */
  int       (*read)(uint8_t *buf, uint16_t length, uint32_t timeout);

  // Buffers
  uint8_t   *txBuffer;
  uint16_t  txBufferSize;
  uint8_t   *rxBuffer;
  uint16_t  rxBufferSize;

  Fingerprint_Packet_t packet;
} Fingerprint_t;



uint8_t AFGR_Init(Fingerprint_t*, 
                  uint8_t *txBuffer, uint16_t txBufferSize, 
                  uint8_t *rxBuffer, uint16_t rxBufferSize);
Fingerprint_Error_t AFGR_VerifyPassword(Fingerprint_t*);
Fingerprint_Error_t AFGR_GetParameters(Fingerprint_t*);

Fingerprint_Error_t AFGR_GetImage(Fingerprint_t*);
Fingerprint_Error_t AFGR_Image2Tz(Fingerprint_t*, Fingerprint_CharBuffer_t);
Fingerprint_Error_t AFGR_CreateModel(Fingerprint_t*);
Fingerprint_Error_t AFGR_CreateModelFromBuffer(Fingerprint_t*, uint8_t *buffer, uint16_t size);
Fingerprint_Error_t AFGR_StoreModel(Fingerprint_t*, uint16_t id);
Fingerprint_Error_t AFGR_LoadModel(Fingerprint_t*, uint16_t id);
int16_t             AFGR_GetModel(Fingerprint_t*, uint8_t *buffer, uint16_t size);
Fingerprint_Error_t AFGR_DeleteModel(Fingerprint_t*, uint16_t id);
Fingerprint_Error_t AFGR_EmptyDatabase(Fingerprint_t*);
Fingerprint_Error_t AFGR_FingerFastSearch(Fingerprint_t*, Fingerprint_CharBuffer_t);
Fingerprint_Error_t AFGR_FingerSearch(Fingerprint_t*, Fingerprint_CharBuffer_t);
Fingerprint_Error_t AFGR_GetTemplateCount(Fingerprint_t*);
Fingerprint_Error_t AFGR_SetPassword(Fingerprint_t*, uint32_t password);
Fingerprint_Error_t AFGR_LEDcontrol(Fingerprint_t*, uint8_t on);



///! Helper class to communicate with and keep state for fingerprint sensors
//class Adafruit_Fingerprint {
//
//  /// The matching location that is set by fingerFastSearch()
//  uint16_t fingerID;
//  /// The confidence of the fingerFastSearch() match, higher numbers are more
//  /// confidents
//  uint16_t confidence;
//  /// The number of stored templates in the sensor, set by getTemplateCount()
//  uint16_t templateCount;
//
//  uint16_t status_reg = 0x0; ///< The status register (set by getParameters)
//  uint16_t system_id = 0x0;  ///< The system identifier (set by getParameters)
//  uint16_t capacity = 64; ///< The fingerprint capacity (set by getParameters)
//  uint16_t security_level = 0; ///< The security level (set by getParameters)
//  uint32_t device_addr =
//      0xFFFFFFFF;             ///< The device address (set by getParameters)
//  uint16_t packet_len = 64;   ///< The max packet length (set by getParameters)
//  uint16_t baud_rate = 57600; ///< The UART baud rate (set by getParameters)
//
//private:
//  uint8_t checkPassword(void);
//  uint32_t thePassword;
//  uint32_t theAddress;
//  uint8_t recvPacket[20];
//
//  Stream *mySerial;
//#if defined(__AVR__) || defined(ESP8266) || defined(FREEDOM_E300_HIFIVE1)
//  SoftwareSerial *swSerial;
//#endif
//  HardwareSerial *hwSerial;
//};

#endif /* ADAFRUIT_FINGERPRINT_CLIB_H */
