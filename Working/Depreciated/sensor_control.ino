#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <ArduinoJson.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
StaticJsonDocument<200> json_doc;


void setup() {
  Serial.begin(115200);
  mlx.begin();  
}

void loop() {
  mlx.readObjectTempF();
  Serial.println();
  json_doc["temp"] = mlx.readObjectTempF();
  serializeJson(json_doc, Serial);
  //serializeJsonPretty(json_doc, Serial);
  delay(500);
}
