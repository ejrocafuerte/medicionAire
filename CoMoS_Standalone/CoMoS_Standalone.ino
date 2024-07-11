/* CoMoS - Comfort Monitoring Station

   written by Mathias Kimmling, Konrad Lauenroth, Prof. Dr.-Ing Sabine Hoffmann
   Department of the Built Environment
   Technische Universität Kaiserslautern

   This sketch is for CoMoS developed by the authors listed above.
   Read more about the development and use cases of the Comfort Monitoring Station at
   http://www.livinglab-smartofficespace.com/
   http://www.livinglab-smartofficespace.com/en/research/heat-and-thermal-comfort/detail/pmv-sensor-stations/
   And more about the authors' at
   https://www.bauing.uni-kl.de/en/gst/home/

   It is designed to run on ESP32 controllers with the following sensors:
   Si7021 - temperature and humidity sensor
   BH1750 - illuminance sensor
   DS18B20+ - temperature sensor
   Rev. C. - wind senser by Modern Device Co.

   Wind speed calculation based on own calibration data.
   Temperature and humidity calibration based on own calibration data.

   MRT calculation based on DIN EN ISO 7726 standard.

   License: CC-BY-NC-SA
   All text above must be included in any redistribution. No commercial use without authors' approval. */


/***** libraries *****/
#include <ESP32WebServer.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "FS.h"
#include <SD.h>
#include "SPIFFS.h"
#include <Wire.h>
#include <BH1750FVI.h>
#include <Adafruit_Si7021.h>
#include <Adafruit_NeoPixel.h>
#include "DallasTemperature.h"
#include "SPI.h"
#include <Time.h>
//#include <TimeLib.h>
#include <RTClib.h>
/***** end of libraries *****/


/***** user configuration *****/
const char* ssid_AP     = "CoMoS_AP";             // variable to set the WiFi SSID of the internal access point, if AP_mode is true
const char* password_AP = "123456789";            // variable to set the associated password
float zeroWindAdjustment = 0.1;             // individual adjustment parameter for wind sensor (standard = 0.1)
/***** end of user configuration *****/


/***** static configuration *****/
#define analogPinForRV    35      // wind sensor RV out on pin 35
#define analogPinForTMP   34      // wind sensor TMP out on pin 34
#define ledPin            17      // RGB LED on pin 17
#define pullupPin         12      // pin 18 used as HIGH level output for pull-up resistor
#define oneWirePin        13      // DS18B20 temperature sensor for globe temperature on pin 19
#define resetPin          14      // pin 14 used as reset trigger
#define powerPinBH1750    25      // pin 25 to power BH1750 sensor
#define powerPinSI7021    26      // pin 26 to power Si7021 sensor
#define powerPinDS18B20   27      // pin 27 to power DS18B20 sensor
#define sdaPin            21      // SDA pin of I2C bus on pin 21
#define sclPin            22      // SCL pin of I2C bus on pin 22
#define slaveSelectPinSD  5       // pin 5  as slave select pin for SD card reader on SPI bus

OneWire ds1(oneWirePin);                                              // instance of OneWire
DallasTemperature sensors1(&ds1);                                     // instance of DallasTemperature
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> ledStrip(1, ledPin);     // instance of Adafruit_NeoPixel
Weather tempSensor;
BH1750FVI lightSensor;                                                // instance of BH1750FVI
RTC_DS3231 rtc;                                                       // instance of RTC_DS3231
/***** end of static configuration *****/


/***** dynamic variable configuration *****/
uint32_t timerSensors = 0;    // variable for sensor read timer
uint32_t timerSend = 0;       // variable for data send timer
uint32_t timerSDLED = 0;
uint32_t blinkIntervalSDLED = 500;
bool pendingResetLED_SD = false;
uint32_t timerClientLED = 0;
uint32_t blinkIntervalClientLED = 200;
bool pendingResetLED_Client = false;
double tempReading = 0;       // variables for sensor readings' temporary storage
double humReading = 0;
double globeReading = 0;
double velReading = 0;
double mrtReading = 0;
double luxReading = 0;
double velReadingAvg = 0;
double luxReadingAvg = 0;
double tempReadingLast = 0;   // variables for last sensor readings, as fall back in case of sensor error
double humReadingLast = 0;
double globeReadingLast = 0;
double velReadingLast = 0;
double mrtReadingLast = 0;
double luxReadingLast = 0;
bool firstReading = true;     // variable for initialisation of min max readings
bool sensorReadNoError = true;
int nReadings = 0;            // variable for counter of data readings and average calculation

const double F_A_temp = 1.00857557;       // constants for calibration factors of Y=A*X+B equation for air temperature, relative humidity, and globe temperature
const double F_B_temp = -0.872682478;      // based on own calibration data - see reference on authors' website
const double F_A_hum = 1.087101959;
const double F_B_hum = -7.173821971;
const double F_A_globe = 0.946524358;
const double F_B_globe = 0.698608493;

float RV_ADunits;                       // variable for analog signal readings of wind sensor
float RV_ADunits_dev1000;               // variable for adjusted analog signal readings of wind sensor
const float T_rev_wind = 21.9;          // constants for wind speed calculation - based on own calibration data - see reference on authors' website
const float v_calc_F1 = 15.36921;
const float v_calc_F2 = 0.01788;
const float v_calc_F3 = -29.04560;
const float v_calc_F4 = 1.01536;

int brightness = 255;         // varibale for RGB LED brightness adjustment (set to max = 255 on boot)
int resetcounter = 0;         // variable for automatic hardware reset in case of errors
int wificounter = 0;          // variable for automatic hardware reset in case of wifi connection timeout

const double diameter = 0.04;     // [[ wird nirgends verwendet? ]]          // diameter of globe temperature bulb in m (standard = 0.04 for table tennis ball)
const double emissivity = 0.95;         // emissivity of globe temp bulb (standard = 0.95 for matte black acrylic paint)
int sensorReadingInterval = 2;          // variable for sensor readings interval, important for online app update, in sec (standard = 2)
bool LEDinterfaceSetting = true;        // variable for LED setting, will be stored in SPIFFS file and loaded from SPIFFS file on boot
/***** end of dynamic variable configuration *****/

/********* SD Card *********/
uint32_t timerSD = 0;
uint32_t dataToSDPeriod = 10;                   // variable for sensor readings to SD-card interval, will be stored in SPIFFS file and loaded from SPIFFS file on boot
char filenameData[] = "/data/000000_C.csv";
File myFile;
/****** end of SD Card *******/

/********* WiFi & Webserver *********/
ESP32WebServer server(80);                        // instance of ESP32WebServer
/********* end of WiFi & Webserver *********/


/********* handle data request function *********/
void handleDataRequest() {
  if (sensorReadNoError) {                  // if no sensor error was detected
    DateTime now = rtc.now();               // get current time from RTC
    String temp = "[\"" + String(tempReading) +  "\",\"" + String(humReading) +  "\",\"" + String(mrtReading) +  "\",\"" + String(velReading) +  "\",\"" + String(luxReading) +  "\",\"";   // assemble JSON string with sensor readings
    if (now.hour() < 10) {                  // if hour is one digit...
      temp += "0";                          // add a zero character to the string first
    }
    temp += String(now.hour()) + ":";       // add hour to the string
    if (now.minute() < 10) {
      temp += "0";
    }
    temp += String(now.minute()) + ":";
    if (now.second() < 10) {
      temp += "0";
    }
    temp += String(now.second()) + "\"]";
    server.send(200, "application/json", temp);     // send string to client, as JSON array
    Serial.println("Sent to client:  " + temp);
  }
}
/********* end of handle data request function *********/


/********* user input procession function *********/
void readUserInputHTTP() {
  if (server.hasArg("localdatetime")) {                   // if the server request holds the argument localdatetime (in the form "2018-09-12T17:35")...
    String argFromUser = server.arg("localdatetime");     // store argument value in a string and in several int values for year, month, day, hour, and minute...
    int yearFromUser = (argFromUser.charAt(0) - '0') * 1000 + (argFromUser.charAt(1) - '0') * 100 + (argFromUser.charAt(2) - '0') * 10 + (argFromUser.charAt(3) - '0');
    int monthFromUser = (argFromUser.charAt(5) - '0') * 10 + (argFromUser.charAt(6) - '0');
    int dayFromUser = (argFromUser.charAt(8) - '0') * 10 + (argFromUser.charAt(9) - '0');
    int hourFromUser = (argFromUser.charAt(11) - '0') * 10 + (argFromUser.charAt(12) - '0');
    int minuteFromUser = (argFromUser.charAt(14) - '0') * 10 + (argFromUser.charAt(15) - '0');
    rtc.adjust(DateTime(yearFromUser, monthFromUser, dayFromUser, hourFromUser, minuteFromUser, 0));      // set the RTC time from the argument
  }
  if (server.hasArg("intreadingstosd")) {                 // if the server request holds the argument intreadingstosd (in the form of an int, but in String format)...
    String argFromUser = server.arg("intreadingstosd");   // store argument value in a string
    dataToSDPeriod = argFromUser.toInt();                 // update global dataToSDPeriod variable with new period setting in int format (in seconds)
    File file = SPIFFS.open("/SDINT.TXT", "w");           // open SDINT.TXT file in write mode (creates the files, if not existing, truncates it to 0 length, if existing)
    file.println(argFromUser);                            // write the argument in the file, in string format (will be read on boot)
    file.close();                                         // close the file
  }
  if (server.hasArg("intreadsensors")) {                  // if the server request holds the argument intreadsensors (in the form of an int, but in String format)...
    String argFromUser = server.arg("intreadsensors");    // store argument value in a string
    sensorReadingInterval = argFromUser.toInt();          // update global sensorReadingInterval variable with new period setting in int format (in seconds)
  }
  if (server.hasArg("ledinterface")) {                    // if the server request holds the argument ledinterface (in String format "true" or "false")...
    String argFromUser = server.arg("ledinterface");      // store argument value in a string
    File file = SPIFFS.open("/LED.TXT", "w");             // open LED.TXT file in write mode (creates the files, if not existing, truncates it to 0 length, if existing)
    if (argFromUser == "true") {                          // if the argument received is "true"...
      LEDinterfaceSetting = true;                         // set global LEDinterfaceSetting variable to true
      setLEDblue();
      file.println(1);                                    // write the "1" in the file, in string format (will be read on boot)
    }
    else if (argFromUser == "false") {                    // if the argument received is "false"...
      LEDinterfaceSetting = false;                        // set global LEDinterfaceSetting variable to false
      setLEDblack();
      file.println(0);                                    // write the "1" in the file, in string format (will be read on boot)
    }
    file.close();                                         // close the file
  }
}
/********* end of user input procession function *********/


/********* send settings to client function *********/
void getSettings() {
  setLEDgreenblue();
  timerClientLED = millis();
  pendingResetLED_Client = true;
  DateTime now = rtc.now();                       // get current time from RTC
  String temp = "[\"";                            // start a string in JSON format
  if (now.day() < 10) {                           // if day is one digit...
    temp += "0";                                  // add a zero character to the string first
  }
  temp += String(now.day()) + ".";                // add day to the string, then do the same for month, year, hour, minute, second...
  if (now.month() < 10) {
    temp += "0";
  }
  temp += String(now.month()) + ".";
  temp += String(now.year()) + " ";
  if (now.hour() < 10) {
    temp += "0";
  }
  temp += String(now.hour()) + ":";
  if (now.minute() < 10) {
    temp += "0";
  }
  temp += String(now.minute()) + ":";
  if (now.second() < 10) {
    temp += "0";
  }
  temp += String(now.second()) + "\",\"";
  temp += String(dataToSDPeriod) + "\",\"";
  if (LEDinterfaceSetting) {                      // if LED interface is on...
    temp += "on\"]";                              // add "on" to the JSON array and close it
  } else {                                        // if LED interface is off...
    temp += "off\"]";                             // add "off" to the JSON array and close it
  }
  server.send(200, "application/json", temp);     // send string to client, as JSON array
  Serial.println("Sent to client:  " + temp);
}
/********* end of send settings to client function *********/


/********* send memory list to client function *********/
void getMemoryList() {
  setLEDgreenblue();
  timerClientLED = millis();
  pendingResetLED_Client = true;
  const int maxFileNumber = 14;                             // define max # of files in list, 14 are implemented in the HTML
  uint64_t totalSpace = SD.totalBytes() / (1024 * 1024);    // read total SD card space, calculate in MB
  uint32_t totalSpaceINT = totalSpace;
  uint64_t usedSpace = SD.usedBytes() / (1024 * 1024);      // read used SD card space, calculate in MB
  uint32_t usedSpaceINT = usedSpace;
  if (!SD.exists("/data")) {                                // if /data/ deirectory doesn't exist...
    SD.mkdir("/data");                                      // create /data/ directory
  }
  File dataDir = SD.open("/data");                          // open /data directory
  String tempStr = "[\"" + String(totalSpaceINT) + "\",\"" + String(usedSpaceINT) + "\",\"";      // start JSON string assembly, include SD space values
  int currentFileNumber = 0;                          // define variables for file listing...
  int lastFileNumber = 0;
  int totalFileNumber = 0;
  String fileNameList[100];                           // string variable array to temporarily store the filenames (up to 100), if more than 100, error will be displayed
  File file = dataDir.openNextFile();                 // open the first file inside the /data/ directory (for counting only in this case)
  while (file) {                                      // while additional files are found and opened...
    fileNameList[totalFileNumber] = file.name();      // add file name to temorary array
    totalFileNumber ++;                               // count up # of files on SD card
    file = dataDir.openNextFile();                    // open the next file
    if (totalFileNumber > 100) {                      // if more than 100 files...
      break;                                          // break from while, stop counting
    }
  }
  currentFileNumber = totalFileNumber - 1;            // set current file number to last entry in the array
  dataDir.close();                                    // close the SD access after counting

  for (int i = _min(maxFileNumber, totalFileNumber); i > 0; i--) {                        // for the number of files available, but not more than 14 (maxFileNumber) times...
    file = SD.open(fileNameList[currentFileNumber]);                                      // open file, last in array
    tempStr += String(file.name()) +  "\",\"" + String((file.size() / 1024)) + "\",\"";   // add file name and size to the JSON array
    currentFileNumber--;                                                                  // count down to open previous file in the array next (results in file list with most recent files on top)
    file.close();                                                                         // close the file
  }
  for (int i = (maxFileNumber - _min(maxFileNumber, totalFileNumber)); i > 0; i--) {      // if less than 14 files with names exist...
    tempStr += "\",\"\",\"";                                                              // input empty JSON variables to reach maxFileNumber and create a JSON array as expected by client
  }
  if (totalFileNumber > 14 && totalFileNumber < 101) {                                    // if there are more than 14 but less than 100 files found... add info message to the JSON string ...
    tempStr += "There are more files in the SD/data/ folder, only the 14 most recently created ones are listed here!";
  } else  if (totalFileNumber > 100) {                                                    // if there are more than 100 files found... add info message to the JSON string ...
    tempStr += "There are more than 100 files in the SD/data/ folder, the file list might not show the most recent ones! Please delete or move some files from SD/data/ folder.";
  } else {                                                                                // else, if not more than 14 files are found...
    tempStr += "\",\"";                                                                   // add a blank entry to JSON array to not show any info message on client
  }
  tempStr += "\"]";                                                                       // close JSON array
  server.send(200, "application/json", tempStr);                                          // send string as JSON array to client
  Serial.println("Sent to client:  " + tempStr);
}
/********* end of send memory list to client function *********/


/********* SETUP FUNCTION *********/
void setup(void) {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  Serial.begin(115200);
  // WiFi.begin(ssid, password);
  pinMode(pullupPin, OUTPUT);             // set pin mode of pullup pin to output
  pinMode(powerPinBH1750, OUTPUT);        // set pin mode of sensor power pin to output
  pinMode(powerPinSI7021, OUTPUT);        // set pin mode of sensor power pin to output
  pinMode(powerPinDS18B20, OUTPUT);       // set pin mode of sensor power pin to output
  pinMode(sdaPin, OUTPUT);                // set pin mode of sensor data pin to output
  pinMode(sclPin, OUTPUT);                // set pin mode of sensor data pin to output
  pinMode(oneWirePin, OUTPUT);            // set pin mode of sensor data pin to output
  digitalWrite(pullupPin, LOW);           // pullup pin LOW to reset sensor
  digitalWrite(powerPinDS18B20, LOW);     // power pin LOW to reset sensor
  digitalWrite(powerPinBH1750, LOW);      // power pin LOW to reset sensor
  digitalWrite(powerPinSI7021, LOW);      // power pin LOW to reset sensor
  digitalWrite(sdaPin, LOW);              // power pin LOW to reset sensor
  digitalWrite(sclPin, LOW);              // power pin LOW to reset sensor
  digitalWrite(oneWirePin, LOW);          // power pin LOW to reset sensor
  delay(2000);
  digitalWrite(pullupPin, HIGH);          // pullup pin HIGH at all times
  digitalWrite(powerPinDS18B20, HIGH);    // power pin HIGH to power sensor
  delay(500);
  digitalWrite(powerPinBH1750, HIGH);     // power pin HIGH to power sensor
  delay(500);
  digitalWrite(powerPinSI7021, HIGH);     // power pin HIGH to power sensor
  delay(1000);

  pinMode(oneWirePin, INPUT);             // sensor pin back to input
  pinMode(sdaPin, INPUT);                 // sensor pin back to input
  for (int i = 0; i < 11; i++) {          // this is a HIGH-LOW-sequence to clear up I2C bus in case it's stuck on slave listening
    digitalWrite(sclPin, HIGH);           // SCL pin HIGH
    delay(1);
    digitalWrite(sclPin, LOW);            // SDA pin HIGH
    delay(1);
  }

  pinMode(sdaPin, INPUT_PULLUP);          // activate pullup resistor on sdaPin
  pinMode(sclPin, INPUT_PULLUP);          // activate pullup resistor on sdaPin
  delay(1000);



  SPIFFS.begin();                         // start instance to access SPIFFS file system
  getSettingsFromSPIFFS();                // call function to read settings from SPIFFS files on boot

  WiFi.softAP(ssid_AP, password_AP);      // start WiFi access point
  IPAddress myIP = WiFi.softAPIP();       // read internal IP address
  Serial.print("AP IP address: ");        // Serial print internal IP address...
  Serial.println(myIP);



  /* Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  */


  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }



  server.on("/admin/renew", handleDataRequest);         // if the client requests /admin/renew, call handleDataRequest function
  server.on("/admin/set", readUserInputHTTP);           // if the client requests /admin/set, call readUserInputHTTP function
  server.on("/admin/getsettings", getSettings);         // if the client requests /admin/getsettings, call getSettings function
  server.on("/admin/getmemory", getMemoryList);         // if the client requests /admin/getmemory, call getMemoryList function

  server.onNotFound([]() {                              // if the client requests any URI
    if (!handleFileRead(server.uri()))                  // call handleFileRead function to send it if it exists on SPIFFS or SD
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  });

  server.begin();                         // start server instance
  Serial.println("HTTP server started");

  Wire.begin();                           // begin I2C connection
  lightSensor.Begin();                    // begin lightSensor instance
  lightSensor.SetMode(Continuous_H2);     // set reading mode of light sensor BH1750: Continuous_H - (default) 1 lx resolution (16 bit), 120ms sampling time --- Continuous_H2 - 0.5 lx resolution (18 bit), 120ms sampling time --- Continuous_L - 4 lx resolution (15 bit), 16ms sampling time
  lightSensor.SetMTReg(254);              // set the measurement time register, which tells the sensor how long to measure. SetSensitivity() uses this same process, but doesn't re-scale the result. Using SetMTReg() results in the same value (re-scaled) but with a different resolution. The range is 31 to 254 with 69 as the default. At 254, resolution is 0.11 lx in _H2 modes
  tempSensor.begin();                     // begin tempSensor instance
  ledStrip.Begin();                       // begin ledStrip instance
  //ledStrip.setBrightness(brightness);     // set RGB LED brightness to initial value
  RgbColor red(brightness, 0, 0);         // define red color variable including brightness
  ledStrip.SetPixelColor(0, red);         // set RGB LED color to red
  if (LEDinterfaceSetting) {
    ledStrip.Show(); // output color to RGB LED
    delay(2000);
  }
  sensors1.begin();                       // begin DS18B20 sensor instance
  portDISABLE_INTERRUPTS();               // disable background interrupts to allow stable OneWire communication
  sensors1.requestTemperatures();         // request temperature sensor DS18B20 update
  portENABLE_INTERRUPTS();                // re-enable interrupts
  delay(1000);

  analogSetWidth(12);                           // set 12 Bit resolution for all analog pins on ESP32 - important to match wind speed calculation and calibration
  analogSetAttenuation((adc_attenuation_t)3);   // set attenuation to -11dB (0=0db (0..1V) 1= 2,5dB; 2=-6dB (0..1V); 3=-11dB) - important to match wind speed calculation and calibration
  SD.begin(slaveSelectPinSD);             // initialize SD card reader
  rtc.begin();                            // initialize RTC external clock, I2C address 0x68    // https://github.com/adafruit/RTClib/blob/master/examples/ds3231/ds3231.ino
  DateTime now = rtc.now();               // read time from RTC and set internal clock...
  setTime(now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
  Serial.println("Sensors on");
  Serial.println("Everything ready");
}
/********* END OF SETUP FUNCTION *********/


/********* LOOP FUNCTION *********/
void loop(void) {
  server.handleClient();                                            // keep server alive and handle connecting clients

  uint32_t minIntervalSensors = _min(sensorReadingInterval, dataToSDPeriod);
  if (millis() - timerSensors >= (minIntervalSensors * 1000)) {  // if timer for sensor readings cycle elapsed...
    timerSensors = millis();                                        // reset timer for sensor readings cycle
    readSensors();                                                  // call function to read sensors
  }
  if (millis() - timerSD >= (dataToSDPeriod * 1000)) {              // if timer for sensor readings to SD card cycle elapsed...
    writeToSD();                                                    // call function to write to SD card
    timerSD = millis();                                             // reset timer
  }
  if (pendingResetLED_SD) {
    if (millis() - timerSDLED >= blinkIntervalSDLED) {              // if timer for sensor readings to SD card cycle elapsed...
      setLEDblue();                                                    // call function to write to SD card
      pendingResetLED_SD = false;
    }
  }
  if (pendingResetLED_Client) {
    if (millis() - timerClientLED >= blinkIntervalClientLED) {              // if timer for sensor readings to SD card cycle elapsed...
      setLEDblue();                                                    // call function to write to SD card
      pendingResetLED_Client = false;
    }
  }







  resetDevice();                                                    // call function to check, if reboot is required
  yield();                                                          // recall ESP's background functions
}
/********* END OF LOOP FUNCTION *********/


/********* get content type function *********/
String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";             // if the file extension is .html, set MIME-type to text/html, proceed for other file types...
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".jss")) return "text/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  return "text/plain";
}
/********* end of get content type function *********/


/********* handle file read function *********/
bool handleFileRead(String path) {
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";         // if a folder is requested, add index.html to the path
  String contentType = getContentType(path);            // call function to get the requested file type and MIME-type
  if (SD.exists(path)) {                                // if the files exists on the SD card...
    File file = SD.open(path, "r");                     // open file in read mode
    size_t sent = server.streamFile(file, contentType); // stream the file content to the client
    file.close();                                       // close the file
    return true;                                        // return true from this function
  }
  if (SPIFFS.exists(path)) {                            // if the files exists in SPIFFS file system...
    File file = SPIFFS.open(path, "r");                 // open file in read mode
    size_t sent = server.streamFile(file, contentType); // stream the file content to the client
    file.close();                                       // close the file
    return true;                                        // return true from this function
  }
  return false;                                         // if file doesn't exist on SD or SPIFFS, return false from this function
}
/********* end of handle file read function *********/


/********* read sensors function *********/
void readSensors() {
  /***** read sensors *****/
  sensorReadNoError = false;

  luxReading = lightSensor.GetLux();            // read light sensor
  //humReading = tempSensor.readHumidity();       // read humidity sensor
  humReading = tempSensor.getRH();
  //tempReading = tempSensor.readTemperature();   // read air temperature sensor
  tempReading = tempSensor.getTemp();


  /***** calculate wind speed *****/
  RV_ADunits = analogRead(analogPinForRV);      // read analog input of wind sensor RV output
  for (int i = 0; i < 7; i++) {                 // read another 7 times to improve reading quality and reduce noise
    RV_ADunits += analogRead(analogPinForRV);
  }
  RV_ADunits /= 8;                              // devide by 8, to get average
  RV_ADunits_dev1000 = RV_ADunits / 1000;       // devide by 1000, to match calibration factors and calculation
  velReading = (2.285 * pow(RV_ADunits_dev1000, 3) - 12.417 * pow(RV_ADunits_dev1000, 2) + 22.831 * RV_ADunits_dev1000 - 14.19) / (T_rev_wind / tempReading) * v_calc_F4 - ((T_rev_wind - tempReading) * v_calc_F1 * pow(RV_ADunits_dev1000, v_calc_F3)) - (T_rev_wind - tempReading) * v_calc_F2 ; // wind speed calculation based on own calibration - see reference on authors' website
  velReading = velReading - zeroWindAdjustment; // adjust zero windspeed
  if (velReading < 0) {
    velReading = 0; // error handling in case of false zeroWindAdjustment setting
  }

  /***** read globe temperature *****/
  portDISABLE_INTERRUPTS();                           // disable background interrupts to allow stable OneWire communication
  if (firstReading) {                                 // DS18B20 needs initialization and calculation time; delivers -127 if both not respected.
    globeReading = sensors1.getTempCByIndex(0);       // get temperature of DS18B20 sensor for globe temperature
    if (globeReading < -50) {                         // check if it's below zero (mostly -127 in case of error. Sensors for indoor use.)
      sensors1.requestTemperatures();                 // request temperature sensor DS18B20 update
      delay(1000);                                    // wait for sensor's internal calculation
      globeReading = sensors1.getTempCByIndex(0);     // get temperature of DS18B20 sensor for globe temperature
    }
    delay(200);
    sensors1.requestTemperatures();                   // request temperature sensor DS18B20 update
    delay(1000);                                      // wait for sensor's internal calculation
    firstReading = false;
  }
  globeReading = sensors1.getTempCByIndex(0);         // get temperature of DS18B20 sensor for globe temperature
  if (globeReading < -50) {                           // error handling again...
    sensors1.requestTemperatures();
    delay(1000);
    globeReading = sensors1.getTempCByIndex(0);
    if (globeReading < 1) {                         // check if it's below zero (mostly -127 in case of error. Sensors for indoor use.)
      globeReading = globeReadingLast;              // use last known value
      resetcounter++;                             // upcount reset counter for error handling
    }
  } else {
    globeReadingLast = globeReading;
  }
  sensors1.requestTemperatures();                 // request temperature sensor DS18B20 update
  portENABLE_INTERRUPTS();                          // re-enable interrupts

  /***** calibration adjustment Y=A*X+B *****/
  tempReading   = F_A_temp  * tempReading   + F_B_temp;     // adjust sensor readings according to calibration data - see reference on authors' website
  humReading    = F_A_hum   * humReading    + F_B_hum;
  globeReading  = F_A_globe * globeReading  + F_B_globe;

  /***** calculate MRT *****/
  mrtReading = pow((pow((globeReading + 273), 4) + (1.1 * 100000000 * pow(velReading, 0.6) / (emissivity * pow(diameter, 0.4)) * (globeReading - tempReading))), 0.25) - 273; // calculate MRT based on DIN EN ISO 7726 standard
  if (isnan(mrtReading)) {    // if above calculation results in not a number NAN
    mrtReading = 0;           // set MRT to 0
  }

  // double mrtReading2 = pow(    pow((globeReading + 273), 4)     +   ( ( ( (1.1 * 100000000 * pow(velReading, 0.6))/(emissivity*pow(diameter,0.4)))*(globeReading - tempReading))), 0.25) - 273;

  /****** adjust LED brightness ******/
  if (luxReading < 255) {               // if illuminance is below 255 lux
    brightness = luxReading;            // set LED brightness variable to current illuminance
    if (brightness < 50) {              // if illuminance is below 50 lux
      brightness = 50;                  // set LED brightness variable to min = 50
    }
  }
  if (luxReading >= 255) {              // for illuminance above 255 lux
    brightness = 255;                   // set LED brightness variable to max = 255
  }
  //ledStrip.setBrightness(brightness);   // set RGB LED brightness to LED brightness variable


  //    luxReading = random(1, 500);
  //    humReading = random(30, 60);
  //    tempReading = random(20, 30);
  //    mrtReading = random(20, 30);
  //    velReading = random(0, 100);
  //    velReading /= 100;

  if (tempReading > 100) {  // here goes all kind of sensor read checks for error values
    brightness = 255;                   // set LED brightness variable to max = 255
    RgbColor red(brightness, 0, 0);     // define red color variable including brightness
    ledStrip.SetPixelColor(0, red);     // set RGB LED color to red
    if (LEDinterfaceSetting) {
      ledStrip.Show(); // output color to RGB LED
    }
    resetcounter++;                             // upcount reset counter for error handling
    Serial.println("Sensor Error");
    Serial.println(resetcounter);
    return;                               // leave the function here...
  }


  //    luxReading = random(1, 500);
  //    humReading = random(30, 60);
  //    tempReading = random(20, 30);
  //mrtReading = random(20, 30);
  //velReading = random(0, 100);
  //velReading /= 100;

  sensorReadNoError = true;  // No errors, write data in variables are okay to send to js
  Serial.println("Sensor readings: " + String(tempReading) + " " + String(humReading) + " " + String(mrtReading) + " " + String(velReading) + " " + String(luxReading) );
}
/********* end of read sensors function *********/


/***** reset function *****/
void resetDevice() {
  if (resetcounter > 5) {                     // if error occurs more than 5 times, reset counter is above 5
    digitalWrite(powerPinSI7021, LOW);        // shut down Si7021 sensor
    digitalWrite(powerPinDS18B20, LOW);       // shut down DS18B20 sensor
    digitalWrite(powerPinBH1750, LOW);        // shut down BH1750 sensor
    pinMode(oneWirePin, OUTPUT);
    digitalWrite(oneWirePin, LOW);            // shut down DS18B20 sensor
    delay(3000);                              // some delay to allow complet power loss on sensors, especially DS18B20+ internal capacitor
    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, LOW);              // Reset Pin to LOW, to reset ESP32
    ESP.restart();                            // reset the ESP (just in case, reset through pin won't work)
  }
}
/***** end of reset function *****/


/***** getFileName function *****/
void getFileName() {
  DateTime now = rtc.now();                           // read current date from RTC for filename update
  filenameData[6] = (now.year() / 10) % 10 + '0';     // read 3rd digit from year()   and update filename
  filenameData[7] = now.year() % 10 + '0';            // read 4th digit from year()   and update filename
  filenameData[8] = now.month() / 10 + '0';           // read 1st digit from month()  and update filename
  filenameData[9] = now.month() % 10 + '0';           // read 2nd digit from month()  and update filename
  filenameData[10] = now.day() / 10 + '0';            // read 1st digit from day()    and update filename
  filenameData[11] = now.day() % 10 + '0';            // read 2nd digit from day()    and update filename
}
/***** end of getFileName function *****/


/********** write to SD function **********/
void writeToSD() {
  getFileName();                                      // call getFileName function to update filenames on date
  DateTime now = rtc.now();                           // read current time and date from RTC
  if (!SD.exists(filenameData))                       // if the file for data is NOT present...
  {
    SD.mkdir("/data");                                // create /data/ directory
    myFile = SD.open(filenameData, FILE_WRITE);       // create and/or open data SD file for writing
    myFile.println("sep=;");                          // define ; as separator in CSV file
    myFile.print("Time [yyyy-MM-dd HH:mm:ss];");      // write legend to sensors file...
    myFile.print("Temperature [degC];");
    myFile.print("Humidity [rH];");
    myFile.print("MRT [degC];");
    myFile.print("Velocity [m/s];");
    myFile.print("Illumination [lx];");
    myFile.println();                                 // write a new line after legend
    myFile.close();                                   // close data file (only one file at a time may be opened)
  }
  myFile = SD.open(filenameData, FILE_APPEND);        // open data SD file for writing, append lines to existing file
  if (myFile) {
    setLEDgreen();
    timerSDLED = millis();
    pendingResetLED_SD = true;
  } else {
    setLEDred();
    timerSDLED = millis();
    pendingResetLED_SD = true;
  }
  myFile.print(now.year()); myFile.print("-"); myFile.print(now.month()); myFile.print("-"); myFile.print(now.day()); myFile.print(" ");        // write current timestamp
  myFile.print(now.hour()); myFile.print(":"); myFile.print(now.minute()); myFile.print(":"); myFile.print(now.second()); myFile.print(";");    // write current timestamp
  myFile.print(tempReading);              myFile.print(";");    // write data to data file...
  myFile.print(humReading);               myFile.print(";");
  myFile.print(mrtReading);               myFile.print(";");
  myFile.print(velReading);               myFile.print(";");
  myFile.print(luxReading);               myFile.print(";");
  myFile.println();                                             // write a new line
  myFile.close();                                               // close data file
  Serial.println("Data sent to SD!");
}
/********** end of write to SD function **********/


/********** get settings from SPIFFS function **********/
void getSettingsFromSPIFFS() {
  if (SPIFFS.exists("/SDINT.TXT")) {                // if the file exists in SPIFFS...
    File file = SPIFFS.open("/SDINT.TXT", "r");     // open the file in read mode
    String argFromFile;                             // define a temporary string variable
    for (int i = 0 ; i < file.size() ; i++) {       // for all characters in the file...
      argFromFile += (char)file.read();             // add character to the string variable
    }
    file.close();                                   // close file
    dataToSDPeriod = argFromFile.toInt();           // update global variable dataToSDPeriod with reading from file
  }

  if (SPIFFS.exists("/LED.TXT")) {                  // if the file exists in SPIFFS...
    File file = SPIFFS.open("/LED.TXT", "r");       // open the file in read mode
    String argFromFile;                             // define a temporary string variable
    for (int i = 0 ; i < file.size() ; i++) {       // for all characters in the file...
      argFromFile += (char)file.read();             // add character to the string variable
    }
    file.close();                                   // close file
    if (argFromFile.toInt() == 0) {                 // if the character read is 0...
      LEDinterfaceSetting = false;                  // set global variable LEDinterfaceSetting to false
    }
    else if (argFromFile.toInt() == 1) {            // if the character read is 0...
      LEDinterfaceSetting = true;                   // set global variable LEDinterfaceSetting to true
    }
  }
}
/********** end of get settings from SPIFFS function **********/


void setLEDgreen() {
  Serial.println("setting LED to green...");
  RgbColor green(0, brightness, 0);     // define red color variable including brightness
  ledStrip.SetPixelColor(0, green);     // set RGB LED color to red
  if (LEDinterfaceSetting) {
    ledStrip.Show(); // output color to RGB LED
  }
}

void setLEDred() {
  Serial.println("setting LED to red...");
  RgbColor red(brightness, 0, 0);     // define red color variable including brightness
  ledStrip.SetPixelColor(0, red);     // set RGB LED color to red
  if (LEDinterfaceSetting) {
    ledStrip.Show(); // output color to RGB LED
  }
}

void setLEDblue() {
  Serial.println("setting LED to blue...");
  RgbColor blue(0, 0, brightness);     // define red color variable including brightness
  ledStrip.SetPixelColor(0, blue);     // set RGB LED color to red
  if (LEDinterfaceSetting) {
    ledStrip.Show(); // output color to RGB LED
  }
}

void setLEDgreenblue() {
  Serial.println("setting LED to greenblue...");
  RgbColor greenblue(0, brightness, brightness);     // define red color variable including brightness
  ledStrip.SetPixelColor(0, greenblue);     // set RGB LED color to red
  if (LEDinterfaceSetting) {
    ledStrip.Show(); // output color to RGB LED
  }
}

void setLEDblack() {
  Serial.println("setting LED to blue...");
  RgbColor black(0, 0, 0);     // define red color variable including brightness
  ledStrip.SetPixelColor(0, black);     // set RGB LED color to red
  ledStrip.Show(); // output color to RGB LED
}
