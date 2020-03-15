#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "esp32-hal-rmt.h"

#define NUM_LEDS  60
#define NUM_CHANNELS  4
#define TOTAL_BITS  8*NUM_CHANNELS*NUM_LEDS
#define PIN_OUT 23

/* From:
https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/RMT/RMTWriteNeoPixel/RMTWriteNeoPixel.ino
https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-rmt.h
*/

rmt_data_t led_data[TOTAL_BITS];
rmt_obj_t* rmt_send = NULL;

const char* ssid = "";
const char* password = "";

void setup() {
  Serial.begin(115200);

  rmt_send = rmtInit(PIN_OUT, true, RMT_MEM_64);
  if(rmt_send == NULL){
    Serial.println("RMT TX init failed");
  }

  float realTick = rmtSetTick(rmt_send, 100);
  Serial.printf("Real tick value set to %fns\n", realTick);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(1000);
    ESP.restart();
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      ESP.restart();
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  int bit_count = 0;
  for(int led = 0; led < NUM_LEDS; led++){
    for(int col_channel = 0; col_channel < NUM_CHANNELS; col_channel++){
      for(int bit = 0; bit < 8; bit++){
        if(col_channel==3){
          led_data[bit_count].level0 = 1;
          led_data[bit_count].duration0 = 6;
          led_data[bit_count].level1 = 0;
          led_data[bit_count].duration1 = 3;
        }else{
          led_data[bit_count].level0 = 1;
          led_data[bit_count].duration0 = 3;
          led_data[bit_count].level1 = 0;
          led_data[bit_count].duration1 = 6;
        }
        bit_count++;
      }
    }
  }
  rmtWrite(rmt_send, led_data, TOTAL_BITS);
  delay(90);
  ArduinoOTA.handle();
}