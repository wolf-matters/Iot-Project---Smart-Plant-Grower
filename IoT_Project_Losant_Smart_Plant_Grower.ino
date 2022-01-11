#include "Freenove_WS2812_Lib_for_ESP32.h"

#define LEDS_COUNT  32
#define LEDS_PIN  4
#define CHANNEL   0

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

u8 m_color[5][3] = { {255, 255,  255}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}, {0, 0, 0} };
int delayval = 100;




#include <arduino-timer.h>

auto timer = timer_create_default(); // create a timer with default settings
Timer<> default_timer; // save as above

// create a timer that can hold 1 concurrent task, with microsecond resolution
// and a custom handler type of 'const char *
Timer<1, micros, const char *> u_timer;


// create a timer that holds 16 tasks, with millisecond resolution,
// and a custom handler type of 'const char *
Timer<16, millis, const char *> t_timer;

int PUMP_on_duration = 1;



// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 16

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


/***************************************************************************
* Example sketch for the BH1750_WE library
* 
* Mode selection / abbreviations:
* CHM:    Continuously H-Resolution Mode
* CHM_2:  Continuously H-Resolution Mode2
* CLM:    Continuously L-Resolution Mode
* OTH:    One Time H-Resolution Mode
* OTH_2:  One Time H-Resolution Mode2
* OTL:    One Time L-Resolution Mode
* 
* Measuring time factor:
* 1.0 ==> Mresuring Time Register = 69
* 0.45 ==> Measuring Time Register = 31
* 3.68 ==> Mesuring Time Register = 254
* 
* Other implemented functions, not used in the example:
* resetDataReg() --> rests Data Register
* powerOn() --> Wake Up!
* powerDown() --> Sleep well, my BH1750
* 
* If you change the measuring time factor for calibration purpose, 
* then you need to devide the light intensity by the measuring time factor 
* 
* Further information can be found on:
* https://wolles-elektronikkiste.de/en/bh1750fvi-gy-30-302-ambient-light-sensor
* or in German:
* https://wolles-elektronikkiste.de/bh1750fvi-lichtsensormodul
***************************************************************************/

///GLOBAL VARIBLES
int brightness = 0 ; 
/////



#include <Wire.h>
#include <BH1750_WE.h>
#define BH1750_ADDRESS 0x23
#define MQTT_MAX_PACKET_SIZE 256

BH1750_WE myBH1750(BH1750_ADDRESS); 
// You may also pass a TwoWire object like wire2 
// BH1750_WE myBH1750(&wire2, BH1750_ADDRESS);


// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"

#define DHTPIN 17     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT11   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 3 (on the right) of the sensor to GROUND (if your sensor has 3 pins)
// Connect pin 4 (on the right) of the sensor to GROUND and leave the pin 3 EMPTY (if your sensor has 4 pins)
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);
/**
 * Example that connects an ESP32 based board to the Losant
 * IoT platform. This example reports state to Losant whenever a button is
 * pressed. It also listens for the "toggle" command to turn the LED on and off.
 *
 * This example assumes the following connections:
 * Button connected to pin 14.
 * LED connected to pin 12.
 *
 * Copyright (c) 2020 Losant. All rights reserved.
 * http://losant.com
 */
#include <WiFi.h>
#include <WiFiClient.h>
#include <Losant.h>
#include <time.h>



// Cert taken from 
// https://github.com/Losant/losant-mqtt-ruby/blob/master/lib/losant_mqtt/RootCA.crt
static const char digicert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";



// WiFi credentials.

/*
// WiFi
const char* WIFI_SSID = "Classroom";    
const char* WIFI_PASS = "ntustclassroom";
*/

/*
// WiFi
const char* ssid = "Factory2_2.4G";                 // Your personal network SSID
const char* wifi_password = "118factory2"; // Your personal network password
*/

///*
const char* WIFI_SSID = "Factory2_2.4G";
const char* WIFI_PASS = "118factory2";

//*/


/*
const char* WIFI_SSID = "TP-LINK_732A3E";
const char* WIFI_PASS = "123456789!@aB";
*/


// Losant credentials.
const char* LOSANT_DEVICE_ID = "61b5ee7c37033edf0865128d";
const char* LOSANT_ACCESS_KEY = "1e175761-91ef-466f-8cf5-7ea82f0711a5";
const char* LOSANT_ACCESS_SECRET = "4391eeb5000ecbaae63c060c62c9e0c2729ea8718777883ac51c186dff9d4e8a";


#define PUMP_PIN 13
#define LED_POWER_PIN 14
#define SOIL_WATER_LEVEL_senorPIN A7


int SOIL_WATER_LEVEL_sensor = SOIL_WATER_LEVEL_senorPIN;    // select the input pin for the potentiometer
int ledPin = 15;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
const int BUTTON_PIN = 32;
const int RELAY1_PIN = PUMP_PIN;
const int RELAY2_PIN = LED_POWER_PIN;

const int LED_PIN = 12;
bool ledState = false;

// initiate the the wifi client
WiFiClient wifiClient;

LosantDevice device(LOSANT_DEVICE_ID);

void toggle() {
  Serial.println("Toggling LED.");
  ledState = !ledState;
  digitalWrite(RELAY1_PIN, ledState ? HIGH : LOW);
}


bool RELAY1_STATE = false;
bool RELAY2_STATE = false;





bool turn_on_PUMP(int duration) {
  //int duration = 5000;
   duration = duration * 1000;
  
  Serial.print("turning on PUMP for: ");
   Serial.println(duration);
   RELAY1_STATE= true;
  digitalWrite(RELAY1_PIN,LOW);
  t_timer.in(duration,turn_off_PUMP_pm,"pump off");
  //t_timer.in(5000, print_message, "delayed five seconds");
    
  return false;
}





// Called whenever the device receives a command from the Losant platform.
void handleCommand(LosantCommand *command) {
  Serial.print("Command received: ");
  Serial.println(command->name);

  // Optional command payload. May not be present on all commands.
     JsonObject payload = *command->payload;

  // Perform action specific to the command received.
  if(strcmp(command->name, "toggle") == 0) {
    toggle();
    /**
    * In Losant, including a payload along with your
    * command is optional. This is an example of how
    * to parse a JSON payload from Losant and print
    * the value of a key called "temperature".
    */
    // int temperature = payload["temperature"];
    // Serial.println(temperature);
  }
  if(strcmp(command->name, "set_BRIGHTNESS") == 0) {
    
    /**
    * In Losant, including a payload along with your
    * command is optional. This is an example of how
    * to parse a JSON payload from Losant and print
    * the value of a key called "temperature".
    */
     brightness = payload["brightness"];
     Serial.print("Setting brightness to ");
     Serial.println(brightness);
     
    strip.setBrightness(brightness);
  }



  if(strcmp(command->name, "turn_on_PUMP") == 0) {
    //toggle();
    /**
    * In Losant, including a payload along with your
    * command is optional. This is an example of how
    * to parse a JSON payload from Losant and print
    * the value of a key called "temperature".
    */
    int duration = payload["duration"];    
    turn_on_PUMP(duration);
    
  }

   if(strcmp(command->name, "turn_off_LIGHT") == 0) {
    /**
    * In Losant, including a payload along with your
    * command is optional. This is an example of how
    * to parse a JSON payload from Losant and print
    * the value of a key called "temperature".
    */
    // int temperature = payload["temperature"];
    // Serial.println(temperature);
    turn_off_LIGHT();
  }
  
  
  if(strcmp(command->name, "turn_on_LIGHT") == 0) {
    /**
    * In Losant, including a payload along with your
    * command is optional. This is an example of how
    * to parse a JSON payload from Losant and print
    * the value of a key called "temperature".
    */
    // int temperature = payload["temperature"];
    // Serial.println(temperature);
    turn_on_LIGHT();
  }



  



  
}







// Set time via NTP, as required for x.509 validation
void setClock() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    now = time(nullptr);
  }
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
}



void connect() {

  // Connect to Wifi.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }  

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  setClock();

  // Connect to Losant.
  Serial.println();
  Serial.print("Connecting to Losant...");

  device.connect(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);

  while(!device.connected()) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected!");
}



void dht_read(){
// Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;}

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));


  // Losant uses a JSON protocol. Construct the simple state object.
  // { "button" : true }
  StaticJsonDocument<200> jsonBuffer;
  JsonObject root = jsonBuffer.to<JsonObject>();
  root["AIR_TEMPERATURE"] = t;
  // Send the state to Losant.
  //device.sendState(root);
  //delay(1);
  root["AIR_HUMIDITY"] = h;
  // Send the state to Losant.
  device.sendState(root);
  


  
}



void read_light_sensor(){
  
  float lightIntensity = myBH1750.getLux();
  Serial.print(F("Lichtstärke: "));
  Serial.print(lightIntensity);
  Serial.println(F(" Lux"));
  // Losant uses a JSON protocol. Construct the simple state object.
  // { "button" : true }
  StaticJsonDocument<200> jsonBuffer;
  JsonObject root = jsonBuffer.to<JsonObject>();
  root["LIGHT_INTENSITY"] = lightIntensity;

  // Send the state to Losant.
  device.sendState(root);






  
}



  


void buttonPressed() {
  Serial.println("Button Pressed!");

  // Losant uses a JSON protocol. Construct the simple state object.
  // { "button" : true }
  StaticJsonDocument<200> jsonBuffer;
  JsonObject root = jsonBuffer.to<JsonObject>();
  root["button"] = true;

  // Send the state to Losant.
  device.sendState(root);
}

int buttonState = 0;




















int read_analog_PIN(int sensorPin){
  volatile int sensorValue = 0;
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  // turn the ledPin on
  //digitalWrite(ledPin, HIGH);
  // send <sensorValue> over serial:
  
  Serial.print("raw soil water: ");
  Serial.println(sensorValue);
  
  // turn the ledPin off:
  //digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds:
  //delay(1);
  return sensorValue;
}

void read_SOIL_WATER_LEVEL(){
  float water_level = 0;
  water_level = 4095 - read_analog_PIN(SOIL_WATER_LEVEL_sensor);
  water_level = water_level/1950 *100;
  Serial.print("corrected water level: ");
  Serial.println(water_level);
  Serial.println("**sending Soil Water Level** ");

  // Losant uses a JSON protocol. Construct the simple state object.
  // { "SOIL_WATER_LEVEL" : value }
  StaticJsonDocument<200> jsonBuffer;
  JsonObject root = jsonBuffer.to<JsonObject>();
  root["SOIL_WATER_LEVEL"] = water_level;

  // Send the state to Losant.
  Serial.println("...sent Soil Water Level ");
  device.sendState(root);
  
}








bool print_message(const char *m) {
  Serial.print("print_message: ");
  Serial.println(m);
  return true; // repeat? true
}




bool turn_off_PUMP_pm(const char *m) {
  Serial.print("print_message: ");
  Serial.println(m);
  RELAY1_STATE= false;
  digitalWrite(RELAY1_PIN, HIGH);
  return true; // repeat? true
}

void turn_on_LIGHT() {
  Serial.println("print_message: turn on light");
  //Serial.println(m);
  RELAY2_STATE= true;
  digitalWrite(RELAY2_PIN, LOW);
  //return true; // repeat? true
}



void turn_off_LIGHT() {
  Serial.println("print_message: turn on light");
  //Serial.println(m);
  RELAY2_STATE= false;
  digitalWrite(RELAY2_PIN, HIGH);
  //return true; // repeat? true
}




size_t repeat_count = 1;
bool repeat_x_times(void *opaque) {
  size_t limit = (size_t)opaque;

  Serial.print("repeat_x_times: ");
  Serial.print(repeat_count);
  Serial.print("/");
  Serial.println(limit);

  return ++repeat_count <= limit; // remove this task after limit reached
}





void WS2812_LED() {
  
strip.setAllLedsColor(m_color[0][0], m_color[0][1], m_color[0][2]);
      strip.show();
      delay(delayval);
   
}









void setup() {


  strip.begin();
  strip.setBrightness(255); 
  
  Serial.begin(115200);
  
  while(!Serial) { }
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(RELAY1_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, HIGH);
  
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, HIGH);
  
  Serial.println(F("DHTxx test!"));

  dht.begin();
   Wire.begin();
   myBH1750.init(); // sets default values: mode = CHM, measuring time factor = 1.0
  // myBH1750.setMode(CLM);  // uncomment if you want to change default values
  // myBH1750.setMeasuringTimeFactor(0.45); // uncomment for selection of value between 0.45 and 3.68

  // Register the command handler to be called when a command is received
  // from the Losant platform.
  device.onCommand(&handleCommand);
  
  //strip.setBrightness(10);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();

  connect();


// call the repeat_x_times function every 1000 millis (1 second)
  timer.every(1000, repeat_x_times, (void *)10);

  // call the print_message function in five seconds
  //t_timer.in(5000, print_message, "delayed five seconds");

  
  // call print_message in 2 seconds, but with microsecond resolution
  //u_timer.in(2000000, print_message, "delayed two seconds using microseconds");
  
}



void read_SOIL_TEMPERATURE(){

  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  float tempC = sensors.getTempCByIndex(0);

  // Check if reading was successful
  if(tempC != DEVICE_DISCONNECTED_C) 
  {
    Serial.print("Temperature for the device 1 (index 0) is: ");
    Serial.println(tempC);
    // Losant uses a JSON protocol. Construct the simple state object.
  // { "SOIL_TEMPERATURE" : value }
    StaticJsonDocument<200> jsonBuffer;
    JsonObject root = jsonBuffer.to<JsonObject>();
    root["SOIL_TEMPERATURE"] = tempC;

  // Send the state to Losant.
  Serial.println("...sent Soil Water Level ");
  device.sendState(root);
  } 
  else
  {
    Serial.println("Error: Could not read temperature data");
  }




  
  
}

  


uint32_t previousMillis = 0;
uint32_t interval =  5000;
bool PUMP_STATUS = false;
//volatile int water_level = 0;

void read_PUMP_STATUS(){
      // Losant uses a JSON protocol. Construct the simple state object.
  // { "PUMP_STATUS" : value }
    StaticJsonDocument<200> jsonBuffer;
    JsonObject root = jsonBuffer.to<JsonObject>();
    root["PUMP_STATUS"] = PUMP_STATUS;

  // Send the state to Losant.
  Serial.println("...sent Soil Water Level ");
  device.sendState(root);
}




 



void loop() {
  



  timer.tick(); // tick the timer
  t_timer.tick();
  u_timer.tick();


  //sensorValue = analogRead(A5);
  //Serial.println(sensorValue);
  delay(100);
  bool toReconnect = false;

  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("Disconnected from WiFi");
    toReconnect = true;
  }

  if(!device.connected()) {
    Serial.println("Disconnected from Losant");
    toReconnect = true;
  }

  if(toReconnect) {
    connect();
  }

  device.loop();


  uint32_t long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    WS2812_LED();
    
    if ( device.connected()== true) {
     Serial.println("Sending Data to Losant");
     read_SOIL_WATER_LEVEL();
     delay(10);
     read_light_sensor();
     delay(10);
     dht_read();
     delay(10);
     read_SOIL_TEMPERATURE();
     
     Serial.println("Reading Data from Losant");  
     
      
    } else {

      Serial.println("Disconnected from Losant");
      delay(1);
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }







  //delay(100);
}
