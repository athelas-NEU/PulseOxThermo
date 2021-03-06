

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
uint16_t irBuffer[40]; //infrared LED sensor data
uint16_t redBuffer[40];  //red LED sensor data
#else
uint32_t irBuffer[40]; //infrared LED sensor data
uint32_t redBuffer[40];  //red LED sensor data
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

  // Init ROS node
  node.initNode();
  node.advertise(pub_temp);
  node.advertise(pub_heart);
  node.advertise(pub_spo2);

  //Wire.begin();
//  Serial.begin(115200);

  
  // Start temp sensor
  mlx.begin();


  // Initialize PulseOx sensor
  particleSensor.begin(Wire);

  // PulseOx Leds
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  //Set onboard LED to blink for diagnostic purposes:
  // pinMode(LED_BUILTIN, OUTPUT);

  /* Removed check for PulseOx sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
      Serial.println(F("MAX30105 was not found. Please check wiring/power."));
      while (1);
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
  bufferLength = 40; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
//        while (particleSensor.available() == false) //do we have new data?
//          particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

//        Serial.print(F("red="));
//        Serial.print(redBuffer[i], DEC);
//        Serial.print(F(", ir="));
//        Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102. Heart rate and SpO2 are calculated every 1 second

for (int k = 0; k < 7; k++)
{
  int wait = 250;
  digitalWrite(readLED, HIGH); //Blink onboard LED with every data read
  delay(wait);                       // wait for a second
  digitalWrite(readLED, LOW); 
  delay(wait);                
}
  
  while (1)
  {
    temp_msg.data = mlx.readObjectTempF();
    heart_msg.data = heartRate;
    spo2_msg.data = spo2;

    pub_temp.publish(&temp_msg);
    pub_heart.publish(&heart_msg);
    pub_spo2.publish(&spo2_msg);

    node.spinOnce();

    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read


    //dumping the first 15 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = bufferLength/4; i < bufferLength; i++)
    {
      redBuffer[i - bufferLength/4] = redBuffer[i];
      irBuffer[i - bufferLength/4] = irBuffer[i];
    }

    //take 15 sets of samples before calculating the heart rate.
    for (byte i = bufferLength*(3/4); i < bufferLength; i++)
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
//            Serial.print(F("red="));
//            Serial.print(redBuffer[i], DEC);
//            Serial.print(F(", ir="));
//            Serial.print(irBuffer[i], DEC);
//      
//                  Serial.print(F(", HR="));
//                  Serial.print(heartRate, DEC);
//      
//            Serial.print(F(", HRvalid="));
//            Serial.print(validHeartRate, DEC);
//      
//                  Serial.print(F(", SPO2="));
//                  Serial.print(spo2, DEC);
//      
//                  Serial.print(F(", SPO2Valid="));
//                  Serial.println(validSPO2, DEC);
      

    }

    //After gathering 15 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }

  // ------------------------ PulseOx sensor code END ------------------------

}
