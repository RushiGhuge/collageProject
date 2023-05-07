#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
// #include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "Wire.h"
#include "Adafruit_VL53L0X.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels

// create an OLED display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// #define PIN_SERVO 23 // Servo motor signel pin -->16

// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 12; // Connects to module's RX
static const uint8_t PIN_MP3_RX = 14; // Connects to module's TX

// Servo myServo;

//TaskHandle_t Task1; //Create Second Loop

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 34
#define SHT_LOX2 16

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X(); //<--Sensor 1
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X(); //<--Sensor 2

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

BluetoothSerial SerialBT; // <-- Bluetooth

char receivedChar; // received value will be stored as CHAR in this variable

const int MR1 = 27; // ESP32 pins (MR=Right Motor) (ML=Left Motor) (1=Forward) (2=Backward)
const int MR2 = 26;
const int ML1 = 25;
const int ML2 = 33;

SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);
// Create the Player object
DFRobotDFPlayerMini player;

void setID()
{
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // initing LOX2
  if (!lox2.begin(LOX2_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
}

void setup()
{
  // this code for the oled for the distance
  Serial.begin(115200);
  // Init serial port for DFPlayer Mini
  softwareSerial.begin(9600);

  // Start communication with DFPlayer Mini
  if (player.begin(softwareSerial))
  {
    Serial.println("OK");

    // Set volume to maximum (0 to 30).
    player.volume(30);
    // Play the first MP3 file on the SD card
    player.play(1);
  }
  else
  {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("failed to start SSD1306 OLED"));
    while (1)
      ;
  }

  delay(2000); // wait two seconds for initializing

  oled.display();
  // display on OLED

  SerialBT.begin("Car2"); // You can change your Bluetooth device name// You c// You can change your Bluetooth device name
  pinMode(MR1, OUTPUT);   // an change your Bluetooth device name // You can change your Bluetooth device name
  pinMode(MR2, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);

  // myServo.attach(PIN_SERVO); // servo pin setup

  //--->Lidar Sensor Setup-->
  // wait until serial port opens for native USB devices
  while (!Serial)
  {
    delay(1);
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));

  Serial.println(F("Starting..."));
  setID();
//
//  xTaskCreatePinnedToCore(
//      Task1code, /* Task function. */
//      "Task1",   /* name of task. */
//      10000,     /* Stack size of task */
//      NULL,      /* parameter of the task */
//      1,         /* priority of the task */
//      &Task1,    /* Task handle to keep track of created task */
//      0);        /* pin task to core 0 */

  // Initialize your task (2nd loop)-->
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

int readFirstSensor() // First Lidar Sensor
{

  lox1.rangingTest(&measure1, false);
  delay(25);
  return measure1.RangeMilliMeter;
}

int readSecondSensor() // Second Lidar Sensor
{
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  delay(25);
  return measure2.RangeMilliMeter;
}

//void Task1code(void *pvParameters)
//{
//  for (;;)
//  {
//    // write you code for 2nd loop here to run endlessly
//    myServo.write(0);
//    //  delay(300);
//
//    myServo.write(60); // left
//    delay(300);
//
//    myServo.write(-60); // right
//    delay(300);
//  }
//}

//  *********************************---> Loop Start <---************************************

void loop()
{

   VL53L0X_RangingMeasurementData_t measure;
  
  int dist_1 = readFirstSensor();
  int dist_2 = readSecondSensor();

  Serial.print(dist_1);
  Serial.print(" ");

  Serial.println(dist_2);

  receivedChar = (char)SerialBT.read();

  if (dist_1 <= 70 || dist_2 <= 70)
  {
    if (receivedChar == 'G')
    {
      Backward();
      Serial.println("backword is calling");
      delay(500);
    }
    else
    {
      Stop();
      player.play(1);
    }

    oled.clearDisplay(); // clear display
    oled.setCursor(0, 0);
    oled.setTextSize(1);      // set text size
    oled.setTextColor(WHITE); // set text color
    // set text
    oled.print("Front : ");
    oled.setTextSize(2);
    oled.print(dist_1);
    oled.setTextSize(1);
    oled.print("cm");
    oled.setTextSize(3);
    oled.println();
    oled.setTextSize(1);
    oled.print("Right : ");
    oled.setTextSize(2);
    oled.print(dist_2);
    oled.setTextSize(1);
    oled.print("cm");
    oled.setTextSize(2);
    oled.println();
    oled.println("WARNING");
    oled.display();
    delay(10);
  }
  else
  {
    oled.clearDisplay(); // clear display
    oled.setCursor(0, 0);
    oled.setTextSize(1);      // set text size
    oled.setTextColor(WHITE); // set text color
    // set text
    oled.print("Front: ");
    oled.setTextSize(2);
    oled.println(dist_1);
    oled.println();
    oled.setTextSize(1);
    oled.print("Right:");
    oled.setTextSize(2);
    oled.println(dist_2);
    oled.display();
    delay(10);
  }

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
    if (receivedChar == 'G')
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

//  <-------------------------<End*Loop>---------------------------->z