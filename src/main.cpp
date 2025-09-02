#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <secret.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define VBATPIN A13
const float V_REF = 3.3;                        // Analog reference voltage (e.g., 5V or 3.3V)
const float R_BITS = 16.0;                      // ADC resolution (bits)
const float ADC_STEPS = (1 << int(R_BITS)) - 1; // Number of steps (2^R_BITS - 1)

const float ADC_VOLT_CORRECTION = 0.0017558602388324;


uint64_t sleepTime = 60 * 1000000; //seconds

#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

// time between sending data to adafruit io, in minutes
#define MESSAGE_WAIT_SEC (15 * 60)

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

AdafruitIO_Feed *huzzah_temperature = io.feed("temperature");
AdafruitIO_Feed *huzzah_humidity = io.feed("humidity");
AdafruitIO_Feed *huzzah_pressure = io.feed("pressure");
AdafruitIO_Feed *huzzah_battery_voltage = io.feed("vbatt");

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // time to get serial running

  esp_err_t result = esp_sleep_enable_timer_wakeup(sleepTime);

  if (result == ESP_OK)
  {
    Serial.println("Timer Wake-Up set successfully as wake-up source.");
  }
  else
  {
    Serial.println("Failed to set Timer Wake-Up as wake-up source.");
  }

  unsigned status;

  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }

  Serial.println();

  io.connect();
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
}

void loop()
{
  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  // put your main code here, to run repeatedly:
  Serial.print("Temperature = ");
  float temp = bme.readTemperature();
  float tempF = temp * 9.0 / 5.0 + 32.0;
  Serial.print(tempF);
  Serial.print(" °F   /   ");
  Serial.print(temp);
  Serial.println(" °C");

  huzzah_temperature->save(tempF, 0, 0, 0, 2);

  Serial.print("Pressure = ");
  float pressure = bme.readPressure() / 100.0F;
  Serial.print(pressure);
  Serial.println(" hPa/mbar");

  huzzah_pressure->save(pressure, 0, 0, 0, 2);

  Serial.print("Humidity = ");
  float humidity = bme.readHumidity();
  Serial.print(humidity);
  Serial.println(" %");

  huzzah_humidity->save(humidity, 0, 0, 0, 2);

  float measuredvbat = analogRead(VBATPIN) * ADC_VOLT_CORRECTION;

  Serial.print("VBat: ");
  Serial.println(measuredvbat);

  huzzah_battery_voltage->save(measuredvbat, 0, 0, 0, 2);

  Serial.println();
  esp_light_sleep_start();     // Enter light sleep
}
