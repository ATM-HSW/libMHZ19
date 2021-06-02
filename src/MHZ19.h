/*   Version: 1.5.3         |  License: LGPLv3  |  Author: JDWifWaf@gmail.com   */
/*   Mbed OS Version 1.0.0  |  License: LGPLv3  |  Author: olaf.hagendorf@hs-wismar.de   */

#ifndef MHZ19_H
#define MHZ19_H

#include "mbed.h"

#define MHZ19_ERRORS 1      // Set to 0 to disable error prints

#define TEMP_ADJUST 38      // This is the value used to adjust the temperature.
                            // Older datsheets use 40, however is likely incorrect.
#define TIMEOUT_PERIOD 500  // Time out period for response (ms)

#define DEFAULT_RANGE 2000  // For range function (sensor works best in this range)

#define MHZ19_DATA_LEN 9    // Data protocl length

// Command bytes -------------------------- //
#define MHZ19_ABC_PERIOD_OFF    0x00
#define MHZ19_ABC_PERIOD_DEF    0xA0

/* enum alias for error code defintions */
enum ERRORCODE {
  RESULT_NULL    = 0,
  RESULT_OK      = 1,
  RESULT_TIMEOUT = 2,
  RESULT_MATCH   = 3,
  RESULT_CRC     = 4,
  RESULT_FILTER  = 5
};

class MHZ19 {
  public:
  /*###########################-Variables-##########################*/

  /* Holds last recieved errorcode from recieveResponse() */
  uint8_t errorCode;

  /* for keeping track of the ABC run interval */
  unsigned long ABCRepeatTimer;

  /*#####################-Initiation Functions-#####################*/

  /* essential begin */
  void begin(BufferedSerial &stream);

  /*########################-Set Functions-##########################*/

  /* Sets Range to desired value*/
  void setRange(int range = 2000);

  /* Sets Span to desired value below 10,000*/
  void zeroSpan(int span = 2000);

    /* Sets "filter mode" to ON or OFF & mode type (see example) */
  void setFilter(bool isON = true, bool isCleared = true);

  /*########################-Get Functions-##########################*/

  /* request CO2 values, 2 types of CO2 can be returned, isLimted = true (command 134) and is Limited = false (command 133) */
  int getCO2(bool isunLimited = true, bool force = true);

  /* returns the "raw" CO2 value of unknown units */
  unsigned int getCO2Raw(bool force = true);

  /* returns Raw CO2 value as a % of transmittance */    //<--- needs work to understand
  float getTransmittance(bool force = true);

  /*  returns temperature using command 134 or 135 if isFloat = true */
  float getTemperature(bool isFloat = false, bool force = true);

  /* reads range using command 153 */
  int getRange();

  /* reads ABC-Status using command 125 / 0x7D */
  bool getABC();

  /* Returns accuracy value if available */
  uint8_t getAccuracy(bool force = true);

  /* not yet implamented */
  uint8_t getPWMStatus();

  /* returns MH-Z19 version using command 160, to the entered array */
  void getVersion(char rVersion[]);

  /* returns background CO2 used by sensor using command 156 */
  int getBackgroundCO2();

  /* returns temperature using command 163 (Note: this library deducts -2 when the value is used) */
  uint8_t getTempAdjustment();

  /* returns last recorded response from device using command 162 */
  uint8_t getLastResponse(uint8_t bytenum);

  /*######################-Utility Functions-########################*/

  /* ensure communication is working (included in begin())*/
  void verify();

  /* disables calibration or sets ABCPeriod */
  void autoCalibration(bool isON = true, uint8_t ABCPeriod = 24);

  /* Calibrates "Zero" (Note: Zero refers to 400ppm for this sensor)*/
  void calibrate();

  /*  Calibrate Backwards compatability */
  void inline calibrateZero(){ calibrate(); };

  /* requests a reset */
  void recoveryReset();

  /* use to show communication between MHZ19 and  Device */
  void printCommunication(bool isDec = true, bool isPrintComm = true);

  private:
  /*###########################-Variables-##########################*/

  /* pointer for Stream class to accept reference for hardware and software ports */
  BufferedSerial* mySerial;

  Timer _timeoutTimer;
  Timer _abcTimer;

  /* alias for command types */
  typedef enum COMMAND_TYPE   {
    RECOVER = 0,      // 0 Recovery Reset
    ABC = 1,          // 1 ABC Mode ON/OFF
    GETABC = 2,       // 2 Get ABC - Status 0x79
    RAWCO2 = 3,       // 3 Raw CO2
    CO2UNLIM = 4,     // 4 Temp for unsigned, CO2 Unlimited
    CO2LIM = 5,       // 5 Temp for signed, CO2 limited
    ZEROCAL = 6,      // 6 Zero Calibration
    SPANCAL = 7,      // 7 Span Calibration
    RANGE = 8,        // 8 Range
    GETRANGE = 9,     // 9 Get Range
    GETCALPPM = 10,   // 10 Get Background CO2
    GETFIRMWARE = 11, // 11 Get Firmware Version
    GETLASTRESP = 12, // 12 Get Last Response
    GETEMPCAL = 13    // 13 Get Temp Calibration
  } Command_Type;

  /* Memory Pool */
  struct mempool {
    struct config  {
      bool ABCRepeat     = false;         // A flag which represents whether autocalibration ABC period was checked
      bool filterMode    = false;         // Flag set by setFilter() to signify is "filter mode" was made active
      bool filterCleared = true;          // Additional flag set by setFiler() to store which mode was selected
      bool printcomm     = false;         // Communication print options
      bool _isDec        = true;          // Holds preferance for communication printing
    } settings;

    uint8_t constructedCommand[MHZ19_DATA_LEN];  // holder for new commands which are to be sent

    struct indata {
      uint8_t CO2UNLIM[MHZ19_DATA_LEN];   // Holds command 133 response values "CO2 unlimited and temperature for unsigned"
      uint8_t CO2LIM[MHZ19_DATA_LEN];     // Holds command 134 response values "CO2 limited and temperature for signed"
      uint8_t RAW[MHZ19_DATA_LEN];        // Holds command 132 response values "CO2 Raw"
      uint8_t STAT[MHZ19_DATA_LEN];       // Holds other command response values such as range, background CO2 etc
    } responses;

  } storage;

  /*######################-Internal Functions-########################*/

  /* Coordinates  sending, constructing and recieving commands */
  void provisioning(Command_Type commandtype, int inData = 0);

  /* Constructs commands using command array and entered values */
  void constructCommand(Command_Type commandtype, int inData = 0);

  /* generates a checksum for sending and verifying incoming data */
  uint8_t getCRC(uint8_t inBytes[]);

  /* Returns what appears to be an offset */     //<----- knowledgable people might have success using against the raw value
  float getTemperatureOffset(bool force = true);

  /* Sends commands to the sensor */
  void write(uint8_t toSend[]);

  /* Call retrieveData to retrieve values from the sensor and check return code */
  uint8_t read(uint8_t inBytes[9], Command_Type commandnumber);

  /* Assigns response to the correct communcation arrays */
  void handleResponse(Command_Type commandtype);

  /* prints sending / recieving messages if enabled */
  void printstream(uint8_t inbytes[9], bool isSent, uint8_t pserrorCode);

  /* Cheks whether time elapse for next ABC OFF cycle has occured */
  void ABCCheck();

  /* converts integers to bytes according to /256 and %256 */
  void makeByte(int inInt, uint8_t *high, uint8_t *low);

  /* converts bytes to integers according to *256 and + value */
  unsigned int makeInt(uint8_t high, uint8_t low);

  void cleanUp();
};
#endif
