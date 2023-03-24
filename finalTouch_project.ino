// #include <Arduino.h>
// #include <MusicDefinitions.h>
// #include <XT_DAC_Audio.h>
#include <Arduino.h>
// #include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "Wire.h"
// #include "Adafruit_VL53L0X.h"

 
// #define PIN_SERVO 23    //Servo motor signel pin -->16
// Servo myServo;

// TaskHandle_t Task1;     

// address we will assign if dual sensor is present
// #define LOX1_ADDRESS 0x30
// #define LOX2_ADDRESS 0x31

// set the pins to shutdown
// #define SHT_LOX1 34
// #define SHT_LOX2 16

// // objects for the vl53l0x
// Adafruit_VL53L0X lox1 = Adafruit_VL53L0X(); //<--Sensor 1
// Adafruit_VL53L0X lox2 = Adafruit_VL53L0X(); //<--Sensor 2

// this holds the measurement
// VL53L0X_RangingMeasurementData_t measure1;
// VL53L0X_RangingMeasurementData_t measure2;

BluetoothSerial SerialBT; // <-- Bluetooth

char receivedChar; // received value will be stored as CHAR in this variable

const int MR1 = 12; // ESP32 pins (MR=Right Motor) (ML=Left Motor) (1=Forward) (2=Backward)
const int MR2 = 14;
const int ML1 = 27;
const int ML2 = 26;

// void setID()
// {
//   // all reset
//   digitalWrite(SHT_LOX1, LOW);
//   digitalWrite(SHT_LOX2, LOW);
//   delay(10);
//   // all unreset
//   digitalWrite(SHT_LOX1, HIGH);
//   digitalWrite(SHT_LOX2, HIGH);
//   delay(10);

//   // activating LOX1 and resetting LOX2
//   digitalWrite(SHT_LOX1, HIGH);
//   digitalWrite(SHT_LOX2, LOW);

//   // initing LOX1
//   if (!lox1.begin(LOX1_ADDRESS))
//   {
//     Serial.println(F("Failed to boot first VL53L0X"));
//     while (1)
//       ;
//   }
//   delay(10);

//   // activating LOX2
//   digitalWrite(SHT_LOX2, HIGH);
//   delay(10);

//   // initing LOX2
//   if (!lox2.begin(LOX2_ADDRESS))
//   {
//     Serial.println(F("Failed to boot second VL53L0X"));
//     while (1)
//       ;
//   }
// }

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("Car2"); // You can change your Bluetooth device name// You c// You can change your Bluetooth device name
  pinMode(MR1, OUTPUT); //an change your Bluetooth device name
// You can change your Bluetooth device name
  pinMode(MR2, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
    
  // myServo.attach(PIN_SERVO);  //servo pin setup

      
  //--->Lidar Sensor Setup-->
  // wait until serial port opens for native USB devices
  // while (!Serial)
  // {
  //   delay(1);
  // }

  // pinMode(SHT_LOX1, OUTPUT);
  // pinMode(SHT_LOX2, OUTPUT);

  // Serial.println(F("Shutdown pins inited..."));

  // digitalWrite(SHT_LOX1, LOW);
  // digitalWrite(SHT_LOX2, LOW);

  // Serial.println(F("Both in reset mode...(pins are low)"));

  // Serial.println(F("Starting..."));
  // setID();

    // Initialize your task (2nd loop)
//   xTaskCreatePinnedToCore(
//     loop2,          // name of the task function
//     "buttonCheck",  // name of the task
//     1000,           // memory assigned for the task
//     NULL,           // parameter to pass if any
//     1,              // priority of task, starting from 0(Highestpriority) *IMPORTANT*( if set to 1 and there is no activity in your 2nd loop, it will reset the esp32)
//     &Task1,         // Reference name of taskHandle variable
//     0);             // choose core (0 or 1

}

void Forward()
{
  digitalWrite(MR1, HIGH); // MOVE FRONT
  digitalWrite(MR2, LOW);  // MOVE BACK
  digitalWrite(ML1, LOW);  // MOVE BACK
  digitalWrite(ML2, HIGH); // MOVE FRONT
}
void Backward()
{
  digitalWrite(MR1, LOW);
  digitalWrite(MR2, HIGH);
  digitalWrite(ML1, HIGH);
  digitalWrite(ML2, LOW);
}
void Left()
{
  digitalWrite(MR1, HIGH);
  digitalWrite(MR2, LOW);
  digitalWrite(ML1, HIGH);
  digitalWrite(ML2, LOW);
}
void Right()
{
  digitalWrite(MR1, LOW);
  digitalWrite(MR2, HIGH);
  digitalWrite(ML1, LOW);
  digitalWrite(ML2, HIGH);
}

void Stop()
{
  digitalWrite(MR1, LOW);
  digitalWrite(MR2, LOW);
  digitalWrite(ML1, LOW);
  digitalWrite(ML2, LOW);
}


// int readFirstSensor() // First Lidar Sensor
// {
  
//   lox1.rangingTest(&measure1, false);
//   delay(25);
//   return measure1.RangeMilliMeter/12;  
// }

// int readSecondSensor() // Second Lidar Sensor
// {
//   lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
//   delay(25);
//   return measure2.RangeMilliMeter/12 ;
// }


// void loop2( void * parameter )
// {
//   for (;;) {
//     // write you code for 2nd loop here to run endlessly
//      myServo.write(0);
//     //  delay(300);

//      myServo.write(60);//left
//      delay(300);

//      myServo.write(-60);//right
//      delay(300); 
//   }
// }

void loop()
{
//   int dist_1 = readFirstSensor();
//   int dist_2 = readSecondSensor();

  // Serial.print(dist_1);
  // Serial.print(" ");

  // Serial.println(dist_2);
 

  delay(10);
  

  receivedChar = (char)SerialBT.read();

  // if (dist_1 <= 10 || dist_2 <= 10)
  // {
  //   if (receivedChar == 'B')
  //   {
  //     Backward();
  //     Serial.println("backword is calling");
  //     delay(500);
  //   }
  //   else
  //   {
  //     Stop();
  //   }
  // }
  
  if (Serial.available())
  {
    SerialBT.write(Serial.read());
  }

  if (SerialBT.available())
  {
    Serial.print("Received:");    // print on serial monitor
    Serial.println(receivedChar); // print on serial monitor

    if (receivedChar == 'F')
    {
      Forward();
    }
    if (receivedChar == 'B')
    {
      Backward();
    }
    if (receivedChar == 'L')
    {
      Left();
    }
    if (receivedChar == 'R')
    {
      Right();
    }
    if (receivedChar == 'S')
    {
      Stop();
    }
delay(10);  
  }
}

//  <-------------------------<End*Loop>---------------------------->
// functions -->
