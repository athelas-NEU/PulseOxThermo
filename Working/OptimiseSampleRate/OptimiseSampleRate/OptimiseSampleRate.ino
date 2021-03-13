// ROS includes START
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
// ROS includes END

#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_MLX90614.h>

#include "heartRate.h"
#include "spo2_algorithm.h"

// ROS msgs and Publishers START
std_msgs::Float32MultiArray temp_msg;
ros::Publisher pub_temp("temp", &temp_msg);

std_msgs::Float32MultiArray heart_msg;
ros::Publisher pub_heart("heart", &heart_msg);

std_msgs::Float32MultiArray spo2_msg;
ros::Publisher pub_spo2("spo2", &spo2_msg);

ros::NodeHandle node;
// ROS msgs and Publisher END

// Temp sensor objects
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
// PulseOx sensor objects
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength = 100; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

const byte RATE_SIZE = 1; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// Led diagnostic blinking to let you know the process has started.
bool ledGoBrrr = false;

void setup()
{
//  Serial.begin(57600);
  // Serial.println("Initializing...");

  temp_msg.layout.dim[0].size = 1;
  temp_msg.layout.data_offset = 0;
  temp_msg.data = (float *)malloc(sizeof(float));
  temp_msg.data_length = 1;

  heart_msg.layout.dim[0].size = 1;
  heart_msg.layout.data_offset = 0;
  heart_msg.data = (float *)malloc(sizeof(float));
  heart_msg.data_length = 1;

  spo2_msg.layout.dim[0].size = 1;
  spo2_msg.layout.data_offset = 0;
  spo2_msg.data = (float *)malloc(sizeof(float));
  spo2_msg.data_length = 1;

  // Init ROS node
  node.initNode();
  node.advertise(pub_temp);
  node.advertise(pub_heart);
  node.advertise(pub_spo2);

//  // Initialize sensor
//  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
//  {
//    Serial.println("MAX30105 was not found. Please check wiring/power. ");
//    while (1);
//  }
//  Serial.println("Place your index finger on the sensor with steady pressure.");

  // Start temp sensor
  mlx.begin();
  
  // Start PulseOx sensor
  particleSensor.begin(Wire, I2C_SPEED_FAST);

  // PulseOx Leds
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  byte ledBrightness = 0xFF; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69; //Options: 69, 118, 215, 411
  int adcRange = 8196; //Options: 2048, 4096, 8192, 16384

  //Configure sensor with these settings
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 

}

void loop()
{

// Blink LED to let operator know that the device is working properly before main loop occurs.
  while(!ledGoBrrr)
  {
    Serial.println("blinking led");
    for (int k = 0; k < 7; k++)
      {
        int wait = 250;
        digitalWrite(readLED, HIGH); //Blink onboard LED with every data read
        delay(wait);                       // wait for a second
        digitalWrite(readLED, LOW); 
        delay(wait);                
      }
      ledGoBrrr = true;
  }
  
  for (byte i = 0 ; i < bufferLength ; i++)
    {
      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample


    }

//  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
//  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  while(1){
      //take 15 sets of samples before calculating the heart rate.
      for (byte i = bufferLength*(3/4); i < bufferLength; i++)
      {
        digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
  
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample
      }
  
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  //
  //  long irValue = particleSensor.getIR();
  //
  //  if (checkForBeat(irValue) == true)
  //  {
  //    //We sensed a beat!
  //    long delta = millis() - lastBeat;
  //    lastBeat = millis();
  //
  //    beatsPerMinute = 60 / (delta / 1000.0);
  //
  //    if (beatsPerMinute < 255 && beatsPerMinute > 20)
  //    {
  //      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
  //      rateSpot %= RATE_SIZE; //Wrap variable
  //
  //      //Take average of readings
  //      beatAvg = 0;
  //      for (byte x = 0 ; x < RATE_SIZE ; x++)
  //        beatAvg += rates[x];
  //      beatAvg /= RATE_SIZE;
  //    }
  //  }
  
      temp_msg.data[0] = mlx.readObjectTempF();
      heart_msg.data[0] = heartRate;
      spo2_msg.data[0] = spo2;
  
      pub_temp.publish(&temp_msg);
      pub_heart.publish(&heart_msg);
      pub_spo2.publish(&spo2_msg);

      node.spinOnce();
  }
}
