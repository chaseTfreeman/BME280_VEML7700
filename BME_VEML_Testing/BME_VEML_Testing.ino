// Adafruit IO Environmental Data Logger 
// Tutorial Link: https://learn.adafruit.com/adafruit-io-air-quality-monitor
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Brent Rubell for Adafruit Industries
// Copyright (c) 2018 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Adafruit IO Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.

// #include "config.h"

/**************************** Sensor Configuration ***************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_VEML7700.h"
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>


// #define WIFI_SSID   "iphone"
// #define WIFI_PASS   "bs6c78pta7wrj"

#define WIFI_SSID   "WholeFoodsMarket"
#define WIFI_PASS   ""

// BME280 Sensor Definitions
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

// Instanciate the sensors
Adafruit_BME280 bme;
Adafruit_VEML7700 veml;
HTTPClient http;    //Declare object of class HTTPClient


/**************************** Example ***************************************/
// Delay between sensor reads, in seconds
#define READ_DELAY 30


// BME280 Data
int altitudeReading = 0;
int humidityReading = 0;
int temperatureReading = 0;
int pressureReading = 0;

void setup() {
  // start the serial connection
  Serial.begin(9600);

  // Allocate the JSON document
  //
  // Inside the brackets, 200 is the capacity of the memory pool in bytes.
  // Don't forget to change this value to match your JSON document.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<200> doc;

  // StaticJsonDocument<N> allocates memory on the stack, it can be
  // replaced by DynamicJsonDocument which allocates in the heap.
  //
  // DynamicJsonDocument doc(200);

  // JSON input string.
  //
  // Using a char[], as shown here, enables the "zero-copy" mode. This mode uses
  // the minimal amount of memory because the JsonDocument stores pointers to
  // the input buffer.
  // If you use another type of input, ArduinoJson must copy the strings from
  // the input to the JsonDocument, so you need to increase the capacity of the
  // JsonDocument.
  char json[] =
      "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  // Fetch values.
  //
  // Most of the time, you can rely on the implicit casts.
  // In other case, you can do doc["time"].as<long>();
  const char* sensor = doc["sensor"];
  long time = doc["time"];
  double latitude = doc["data"][0];
  double longitude = doc["data"][1];

  // Print values.
  Serial.println(sensor);
  Serial.println(time);
  Serial.println(latitude, 6);
  Serial.println(longitude, 6);

  //wifi setup
  WiFi.begin("WIFI_SSID", "WIFI_PASS");   //WiFi connection
 
  while (WiFi.status() != WL_CONNECTED) {  //Wait for the WiFI connection completion
 
    delay(500);
    Serial.println("Waiting for connection");
 
  }
 
  Serial.println("WIFI Connected!");
 

  Serial.println("BME VEML test");

  if (!veml.begin()) {
    Serial.println("VEML7700 Sensor not found");
    while (1);
  }
  Serial.println("VEML7700 Sensor found");

  veml.setGain(VEML7700_GAIN_1);
  veml.setIntegrationTime(VEML7700_IT_800MS);

  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }

  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }
  //veml.powerSaveEnable(true);
  //veml.setPowerSaveMode(VEML7700_POWERSAVE_MODE4);

  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);

  Serial.print("Lux: "); Serial.println(veml.readLux());
  Serial.print("White: "); Serial.println(veml.readWhite());
  Serial.print("Raw ALS: "); Serial.println(veml.readALS());

  uint16_t irq = veml.interruptStatus();
  if (irq & VEML7700_INTERRUPT_LOW) {
    Serial.println("** Low threshold"); 
  }
  if (irq & VEML7700_INTERRUPT_HIGH) {
    Serial.println("** High threshold"); 
  }

  // wait for serial monitor to open
  //  while (!Serial);

  Serial.println("Setting up BME280 ");

  //set up BME280
  setupBME280();

}

void loop() {
  Serial.println("***NEW BLOCK***");


  Serial.println("Reading Sensors...");

  // Read the temperature from the BME280
  temperatureReading = bme.readTemperature();

  Serial.println("***VEML7700***");
  Serial.print("Lux: "); Serial.println(veml.readLux());
  Serial.print("White: "); Serial.println(veml.readWhite());
  Serial.print("Raw ALS: "); Serial.println(veml.readALS());

  uint16_t irq = veml.interruptStatus();
  if (irq & VEML7700_INTERRUPT_LOW) {
    Serial.println("** Low threshold"); 
  }
  if (irq & VEML7700_INTERRUPT_HIGH) {
    Serial.println("** High threshold"); 
  }

  Serial.println("***BME2800***");
  // convert from celsius to degrees fahrenheit
  temperatureReading = temperatureReading * 1.8 + 32;
  
  Serial.print("Temperature = "); Serial.print(temperatureReading); Serial.println(" *F");

  // Read the pressure from the BME280
  pressureReading = bme.readPressure() / 100.0F;
  Serial.print("Pressure = "); Serial.print(pressureReading); Serial.println(" hPa");

  // Read the altitude from the BME280
  altitudeReading = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("Approx. Altitude = "); Serial.print(altitudeReading); Serial.println(" m");
  
  // Read the humidity from the BME280
  humidityReading = bme.readHumidity();
  Serial.print("Humidity = "); Serial.print(humidityReading); Serial.println("%");

 // POST REQUEST TEST
  // if(WiFi.status()== WL_CONNECTED){   //Check WiFi connection status
  
  //  http.begin("https://play.dhis2.org/2.33.2/api/categoryOptions");      //Specify request destination
  //  http.addHeader("Content-Type", "application/json");  //Specify content-type header
 
  //  int httpCode = http.POST('{\"code\":\"test_arduino\",\"name\":\"test_arduino\",\"shortName\":\"test_arduino\",\"organisationUnits":[{\"id\":\"ImspTQPwCqd\"}]}');   //Send the request
  //  String payload = http.getString();   //Get the response payload
 
  //  Serial.println(httpCode);   //Print HTTP return code
  //  Serial.println(payload);    //Print request response payload
 
  //  http.end();  //Close connection
 
//  }else{
 
//     Serial.println("Error in WiFi connection");   
 
//  }
 
  delay(5000);  //Send a request every 30 seconds
 

  // delay the polled loop
  // delay(READ_DELAY * 5000);
}

// Set up the BME280 sensor
void setupBME280() {
  bool status;
  status = bme.begin();
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Serial.println("BME Sensor is set up!");
}
