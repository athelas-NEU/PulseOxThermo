
/*
  Optical SP02 Detection (SPK Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 19th, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This demo shows heart rate and SPO2 levels.

  It is best to attach the sensor to your finger using a rubber band or other tightening 
  device. Humans are generally bad at applying constant pressure to a thing. When you 
  press your finger against the sensor it varies enough to cause the blood in your 
  finger to flow differently which causes the sensor readings to go wonky.

  This example is based on MAXREFDES117 and RD117_LILYPAD.ino from Maxim. Their example
  was modified to work with the SparkFun MAX30105 library and to compile under Arduino 1.6.11
  Please see license file for more info.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

// ROS includes START
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
// ROS includes END

#include <Wire.h>
#include <Adafruit_MLX90614.h>

#include "MAX30105.h"
#include "spo2_algorithm.h"


// ROS msgs and Publishers START

std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temp", &temp_msg);

std_msgs::Float32 heart_msg;
ros::Publisher pub_heart("heart", &heart_msg);

std_msgs::Float32 spo2_msg;
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

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

void setup()
{
  //Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  
  // Start temp sensor
  mlx.begin();  

  // PulseOx Leds
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize PulseOx sensor
  particleSensor.begin(Wire, I2C_SPEED_FAST);

  // Init ROS node
  node.initNode();
  node.advertise(pub_temp);
  node.advertise(pub_heart);
  node.advertise(pub_spo2);

  //pinmode(a0, OUTPUT);
  //digitalWrite(a0, HIGH)l;
  
/* Removed check for PulseOx sensor 
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
*/

/*  
  PulseOx code asking for confirmation before beginning has been turned off
  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0); //wait until user presses a key 
  {
    Serial.read();
  }
*/

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop()
{
	
	
  // ------------------------ PulseOx sensor code START ------------------------
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

//    Serial.print(F("red="));
//    Serial.print(redBuffer[i], DEC);
//    Serial.print(F(", ir="));
//    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102. Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
     /*
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
     */
     
      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
//      Serial.print(F("red="));
//      Serial.print(redBuffer[i], DEC);
//      Serial.print(F(", ir="));
//      Serial.print(irBuffer[i], DEC);
//
//      Serial.print(F(", HR="));
//      Serial.print(heartRate, DEC);
//
//      Serial.print(F(", HRvalid="));
//      Serial.print(validHeartRate, DEC);
//
//      Serial.print(F(", SPO2="));
//      Serial.print(spo2, DEC);
//
//      Serial.print(F(", SPO2Valid="));
//      Serial.println(validSPO2, DEC);

      // Added Temp sensor to print out. Not ideal but serves for now at least.
//  	  Serial.print(F(", TEMP="));
//  	  Serial.println( mlx.readObjectTempF() );

      temp_msg.data = mlx.readObjectTempF();
      heart_msg.data = heartRate;
      spo2_msg.data = spo2;

      pub_temp.publish(&temp_msg);
      pub_heart.publish(&heart_msg);
      pub_spo2.publish(&spo2_msg);

      /*  This likely only needed for subscribers. Since this program only publishes can likely be removed.
      */
	  
      // node.spinOnce();
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
  
  // ------------------------ PulseOx sensor code END ------------------------
  
}
