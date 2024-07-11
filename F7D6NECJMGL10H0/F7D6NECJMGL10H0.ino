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
#include <Wire.h>
#include <BH1750FVI.h>
#include "Adafruit_Si7021.h"
#include "Adafruit_NeoPixel.h"
#include "DallasTemperature.h"
#include "SPI.h"
#include "WiFi.h"
/***** end of libraries *****/


/***** user configuration *****/
int nodeID = 1;                             // ID of this sensor station (standard = 1)
const int sendCycleTime = 10;               // cycle time to send sensor data to database (in seconds, standard = 10)
const int readSensorTime = 1;               // cycle time to read sensor data for average calculation (in seconds, standard = 1)
const char* ssid = "YourWiFiName";          // setup variable for WiFi SSID
const char* password = "YourWiFiPassword";  // setup variable for WiFi password
IPAddress server_addr(192, 168, 0, 0);      // IP of your PHP/MariaDB server
IPAddress gateway(192, 168, 0, 0);          // gateway IP of your network gateway
IPAddress subnet(255, 255, 255, 0);         // subnet mask of your network
const double diameter = 0.04;               // diameter of globe temperature bulb in m (standard = 0.04 for table tennis ball)
const double emissivity = 0.95;             // emissivity of globe temp bulb (standard = 0.95 for matte black acrylic paint)
float zeroWindAdjustment = 0.1;             // individual adjustment parameter for wind sensor (standard = 0.1)
/***** end of user configuration *****/


/***** static configuration *****/
#define analogPinForRV    35      // wind sensor RV out on pin 35
#define ledPin            17      // RGB LED on pin 17
#define pullupPin         18      // pin 18 used as HIGH level output for pull-up resistor
#define oneWirePin        19      // DS18B20 temperature sensor for globe temperature on pin 19
#define resetPin          14      // pin 14 used as reset trigger
#define powerPinBH1750    25      // pin 25 to power BH1750 sensor
#define powerPinSI7021    26      // pin 26 to power Si7021 sensor
#define powerPinDS18B20   27      // pin 27 to power DS18B20 sensor
#define sdaPin            21      // SDA pin of I2C bus on pin 21
#define sclPin            22      // SCL pin of I2C bus on pin 22

OneWire ds1(oneWirePin);                                              // instance of OneWire
DallasTemperature sensors1(&ds1);                                     // instance of DallasTemperature
Adafruit_NeoPixel ledStrip = Adafruit_NeoPixel(1, ledPin, NEO_GRB);   // instance of Adafruit_NeoPixel
Adafruit_Si7021 tempSensor = Adafruit_Si7021();                       // instance of Adafruit_Si7021
BH1750FVI lightSensor;                                                // instance of BH1750FVI
WiFiClient client;                                                    // instance of WiFiClient
/***** end of static configuration *****/


/***** dynamic variable configuration *****/
uint32_t timerSensors = 0;    // variable for sensor read timer
uint32_t timerSend = 0;       // variable for data send timer
double tempReading = 0;       // variables for sensor readings' temporary storage
double humReading = 0;
double globeReading = 0;
double velReading = 0;
double mrtReading = 0;
double luxReading = 0;
double velMinReading = 0;
double velMaxReading = 0;
double luxMinReading = 0;
double luxMaxReading = 0;
double velReadingAvg = 0;
double luxReadingAvg = 0;
double tempReadingLast = 0;   // variables for last sensor readings, as fall back in case of sensor error
double humReadingLast = 0;
double globeReadingLast = 0;
double velReadingLast = 0;
double mrtReadingLast = 0;
double luxReadingLast = 0;
bool firstReading = true;     // variable for initialisation of min max readings
int nReadings = 0;            // variable for counter of data readings and average calculation

const double F_A_temp = 1.001474;       // constants for calibration factors of Y=A*X+B equation for air temperature, relative humidity, and globe temperature
const double F_B_temp = -0.732693;      // based on own calibration data - see reference on authors' website
const double F_A_hum = 1.024386;
const double F_B_hum = -4.222511;
const double F_A_globe = 0.933372;
const double F_B_globe = 1.027721;

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
/***** end of dynamic variable configuration *****/


/***** function to read Sensors, calculate values, and update RGB LED brightness *****/
void readSensors() {
  /***** read sensors *****/
  luxReading = lightSensor.GetLightIntensity();            // read light sensor
  humReading = tempSensor.readHumidity();       // read humidity sensor
  tempReading = tempSensor.readTemperature();   // read air temperature sensor

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
    velReading = 0;                             // error handling in case of false zeroWindAdjustment setting
  }

  /***** read globe temperature *****/
  portDISABLE_INTERRUPTS();                          // disable background interrupts to allow stable OneWire communication
  if (firstReading) {                                // DS18B20 needs initialization and calculation time; delivers -127 if both not respected.
    globeReading = sensors1.getTempCByIndex(0);       // get temperature of DS18B20 sensor for globe temperature
    if (globeReading < 1) {                           // check if it's below zero (mostly -127 in case of error. Sensors for indoor use.)
      sensors1.requestTemperatures();                 // request temperature sensor DS18B20 update
      delay(1000);                                    // wait for sensor's internal calculation
      globeReading = sensors1.getTempCByIndex(0);     // get temperature of DS18B20 sensor for globe temperature
    }
    delay(200);
    sensors1.requestTemperatures();                   // request temperature sensor DS18B20 update
    delay(1000);                                      // wait for sensor's internal calculation
  }
  globeReading = sensors1.getTempCByIndex(0);         // get temperature of DS18B20 sensor for globe temperature
  if (globeReading < 1) {                             // error handling again...
    sensors1.requestTemperatures();
    delay(1000);
    globeReading = sensors1.getTempCByIndex(0);
    if (globeReading < 1) {                         // check if it's below zero (mostly -127 in case of error. Sensors for indoor use.)
      globeReading = globeReadingLast;              // use last known value
    }
    sensors1.requestTemperatures();                 // request temperature sensor DS18B20 update
  }
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
  ledStrip.setBrightness(brightness);   // set RGB LED brightness to LED brightness variable
}
/***** end of function to read Sensors, calculate values, and update RGB LED brightness *****/


/***** function to send data *****/
void sendData() {
  velReadingAvg = velReadingAvg / nReadings;  // calculate averages of sensor readings
  luxReadingAvg = luxReadingAvg / nReadings;

  if (tempReading > 100) {
    ledStrip.setBrightness(255);                // set RGB LED brightness to max
    ledStrip.setPixelColor(0, 0, 255, 255);     // set RGB LED color to greenblue
    ledStrip.show();                            // output color to RGB LED
    resetcounter++;                             // upcount reset counter for error handling
  }

  if (globeReading <= 0) {
    ledStrip.setBrightness(255);                // set RGB LED brightness to max
    ledStrip.setPixelColor(0, 255, 0, 255);     // set RGB LED color to purple
    ledStrip.show();                            // output color to RGB LED
    resetcounter++;                             // upcount reset counter for error handling
  }

  if ( (tempReading < 100) && (globeReading > 0) && (humReading < 100) ) {    // this is another error handling, in case bus communication is down, sensors will return false readings
    if (client.connect(server_addr, 80 )) {     // check if connection to PHP server was successful
      client.print("GET /sensor.php?ID=");      // call sensor-PHP file on server and write values
      client.print(nodeID);
      client.print("&TEMP=");
      client.print(tempReading);
      client.print("&HUM=");
      client.print(humReading);
      client.print("&GLOBE=");
      client.print(globeReading);
      client.print("&VEL=");
      client.print(velReadingAvg);
      client.print("&VELMIN=");
      client.print(velMinReading);
      client.print("&VELMAX=");
      client.print(velMaxReading);
      client.print("&MRT=");
      client.print(mrtReading);
      client.print("&ILLUM=");
      client.print(luxReadingAvg);
      client.print("&ILLUMMIN=");
      client.print(luxMinReading);
      client.print("&ILLUMMAX=");
      client.print(luxMaxReading);
      client.print(" HTTP/1.0\r\n");
      client.println();
      client.stop();                          // terminate PHP server connection
      tempReading = 0;                        // reset sensor readings variables
      humReading = 0;
      globeReading = 0;
      velReadingAvg = 0;
      mrtReading = 0;
      luxReadingAvg = 0;
      nReadings = 0;
      luxMaxReading = 0;
      luxMinReading = 0;
      velMaxReading = 0;
      velMinReading = 0;
      firstReading = true;                      // set first reading true for special min max values handling
      resetcounter = 0;                         // set reset counter back to 0
      ledStrip.setPixelColor(0, 0, 255, 0);     // set RGB LED color to green
      ledStrip.show();                          // output color to RGB LED
      delay(600);                               // wait 600 ms
      ledStrip.setPixelColor(0, 0, 0, 255);     // set RGB LED color back to blue
      ledStrip.show();                          // output color to RGB LED
    } else {
      ledStrip.setPixelColor(0, 255, 0, 0);     // set RGB LED color to red
      ledStrip.show();                          // output color to RGB LED
    }
  } else {                                      // if bus communication is down
    ledStrip.setPixelColor(0, 255, 0, 0);       // set RGB LED color to red
    ledStrip.show();                            // output color to RGB LED
    resetcounter++;                             // upcount reset counter
    portDISABLE_INTERRUPTS();                   // disable background interrupts to allow stable OneWire communication
    sensors1.requestTemperatures();
    portENABLE_INTERRUPTS();                    // re-enable interrupts
  }
}
/***** end of function to send data *****/


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


/***** setup *****/
void setup() {
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
  WiFi.begin(ssid, password);             // begin a wifi connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    wificounter++;
    if (wificounter > 30) {
      pinMode(resetPin, OUTPUT);
      digitalWrite(resetPin, LOW);        // Reset Pin to LOW, to reset ESP32
      ESP.restart();                      // reset the ESP (just in case, reset through pin won't work)
    }
  }
  Wire.begin();                           // begin I2C connection
  lightSensor.begin();                    // begin lightSensor instance
  lightSensor.SetMode(Continuous_H_resolution_Mode2);     // set reading mode of light sensor BH1750: Continuous_H - (default) 1 lx resolution (16 bit), 120ms sampling time --- Continuous_H2 - 0.5 lx resolution (18 bit), 120ms sampling time --- Continuous_L - 4 lx resolution (15 bit), 16ms sampling time
  //lightSensor.SetMTReg(254); funcion no existe en libreria              // set the measurement time register, which tells the sensor how long to measure. SetSensitivity() uses this same process, but doesn't re-scale the result. Using SetMTReg() results in the same value (re-scaled) but with a different resolution. The range is 31 to 254 with 69 as the default. At 254, resolution is 0.11 lx in _H2 modes
  tempSensor.begin();                     // begin tempSensor instance
  ledStrip.begin();                       // begin ledStrip instance
  ledStrip.setBrightness(brightness);     // set RGB LED brightness to initial value
  ledStrip.setPixelColor(0, 255, 0, 0);   // set RGB LED color to red
  ledStrip.show();                        // output color to RGB LED
  sensors1.begin();                       // begin DS18B20 sensor instance
  portDISABLE_INTERRUPTS();               // disable background interrupts to allow stable OneWire communication
  sensors1.requestTemperatures();         // request temperature sensor DS18B20 update
  portENABLE_INTERRUPTS();                // re-enable interrupts
  delay(1000);

  //analogSetWidth(12);                           // set 12 Bit resolution for all analog pins on ESP32 - important to match wind speed calculation and calibration
  analogSetAttenuation((adc_attenuation_t)3);   // set attenuation to -11dB (0=0db (0..1V) 1= 2,5dB; 2=-6dB (0..1V); 3=-11dB) - important to match wind speed calculation and calibration
}
/***** end of setup *****/


/***** loop *****/
void loop() {
  /***** sensor readings cycle *****/
  if (millis() - timerSensors >= (readSensorTime * 1000)) {   // if timer for sensor readings cycle elapsed
    timerSensors = millis();                                  // reset timer for sensor readings cycle
    readSensors();                                            // call read sensors function
    if (firstReading) {                                       // if first reading after data sent or boot, set min & max to first sensor readings
      velMinReading = velReading;
      velMaxReading = velReading;
      luxMinReading = luxReading;
      luxMaxReading = luxReading;
      firstReading = false;                                   // reset the initial reading variable
    }
    if (velMinReading > velReading) {
      velMinReading = velReading;  // min & max determination
    }
    if (velMaxReading < velReading) {
      velMaxReading = velReading;
    }
    if (luxMinReading > luxReading) {
      luxMinReading = luxReading;
    }
    if (luxMaxReading < luxReading) {
      luxMaxReading = luxReading;
    }

    tempReadingLast = tempReading;                  // store current readings for next iteration, needed if sensor error occurs
    humReadingLast = humReading;
    globeReadingLast = globeReading;
    mrtReadingLast = mrtReading;

    velReadingAvg = velReadingAvg + velReading;     // add up sensor readings for average calculation
    luxReadingAvg = luxReadingAvg + luxReading;
    nReadings++;                                    // add up counter variable for average calculation +1
  }

  /***** send data cycle *****/
  if (millis() - timerSend > (sendCycleTime * 1000)) {  // if timer for send data cycle elapsed
    timerSend = millis();                               // reset timer for send data cycle
    sendData();                                         // call sendData function (includes average calculation)
  }
  if (WiFi.status() != WL_CONNECTED) {                  // check if Wifi is still available
    delay(2000);
    resetcounter++;
  }
  resetDevice();                                        // Checks if Device needs to be reset
  yield();                                              // recall ESP's background functions
}
/***** end loop *****/
