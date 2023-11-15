#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include <bluefruit.h>
//#include "Keyboard.h"

BLEDis bledis;
BLEHidAdafruit blehid;

#define VERSION "0.1"
//#define LED_OFF

#define APDS9960_INT    7 // Needs to be an interrupt pin

#define upLedPin 27
#define downLedPin 15
#define leftLedPin 2
#define rightLedPin 4
#define plusLedPin 5
#define minusLedPin 30
#define correctLedPin 16
#define wrongLedPin 3

SparkFun_APDS9960 apds = SparkFun_APDS9960();

int isr_flag = 0;

bool upLedActive = false;
bool downLedActive = false;
bool rightLedActive = false;
bool leftLedActive = false;
bool plusLedActive = false;
bool minusLedActive = false;

uint8_t proximity_data = 0;

/*
  void lightUpLed(pin)
  {
  digitalWrite(upLedPin,HIGH);
  delay(500);
  digitalWrite(upLedPin,LOW);
  }

  void lightDownLed(pin)
  {
  digitalWrite(downLedPin,HIGH);
  delay(500);
  digitalWrite(downLedPin,LOW);
  }
  void lightLeftLed(pin)
  {
  digitalWrite(leftLedPin,HIGH);
  delay(500);
  digitalWrite(leftLedPin,LOW);
  }
  void lightRightLed(pin)
  {
  digitalWrite(rightLedPin,HIGH);
  delay(500);
  digitalWrite(rightLedPin,LOW);
  }
*/

void setup()
{
  Serial.begin(115200);

  pinMode(upLedPin, OUTPUT);
  digitalWrite(upLedPin, LOW);

  pinMode(downLedPin, OUTPUT);
  digitalWrite(downLedPin, LOW);

  pinMode(leftLedPin, OUTPUT);
  digitalWrite(leftLedPin, LOW);

  pinMode(rightLedPin, OUTPUT);
  digitalWrite(rightLedPin, LOW);

  pinMode(plusLedPin, OUTPUT);
  digitalWrite(plusLedPin, LOW);

  pinMode(minusLedPin, OUTPUT);
  digitalWrite(minusLedPin, LOW);

  pinMode(wrongLedPin, OUTPUT);
  digitalWrite(wrongLedPin, LOW);

  pinMode(correctLedPin, OUTPUT);
  digitalWrite(correctLedPin, LOW);

  //pinMode(LED_BUILTIN, OUTPUT);

  // Set interrupt pin as input
  pinMode(APDS9960_INT, INPUT);

  // Initialize Serial port

  Serial.println();
  Serial.println("Go to your phone's Bluetooth settings to pair your device");
  Serial.println("then open an application that accepts keyboard input");
  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("BlueFruit APDS9960");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Emagine Tests");
  bledis.setModel("Gesture Control 52");
  bledis.begin();

  blehid.begin();

  startAdv();

  showHeader();

  Serial.println();
  Serial.println(F("--------------------------------"));
  Serial.println(F("SparkFun APDS-9960 - GestureTest"));
  Serial.println(F("--------------------------------"));

  // Initialize interrupt service routine
  attachInterrupt(digitalPinToInterrupt(APDS9960_INT), interruptRoutine, FALLING);

  // Initialize APDS-9960 (configure I2C and initial values)
  if (apds.init()) {
    Serial.println(F("APDS-9960 initialization complete"));
  }
  else
  {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

  // Start running the APDS-9960 gesture sensor engine
  if (apds.enableGestureSensor(true)) {
    Serial.println(F("Gesture sensor is now running"));
  }
  else
  {
    Serial.println(F("Something went wrong during gesture sensor init!"));
  }
}

void startAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);

  Bluefruit.Advertising.addService(blehid);

  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop()
{
  if (isr_flag == 1)
  {
    detachInterrupt(digitalPinToInterrupt(APDS9960_INT));
    handleGesture();
    isr_flag = 0;
    attachInterrupt(digitalPinToInterrupt(APDS9960_INT), interruptRoutine, FALLING);
  }

  if (!apds.readProximity(proximity_data))
  {
    Serial.println("Error reading proximity value");
  }

  if (upLedActive)
  {
    digitalWrite(upLedPin, HIGH);
    digitalWrite(correctLedPin, HIGH);
    delay(500);
    digitalWrite(correctLedPin, LOW);
    digitalWrite(upLedPin, LOW);
    upLedActive = false;
  }
  else if (downLedActive)
  {
    digitalWrite(downLedPin, HIGH);
    digitalWrite(correctLedPin, HIGH);
    delay(500);
    digitalWrite(correctLedPin, LOW);
    digitalWrite(downLedPin, LOW);
    downLedActive = false;
  }
  else if (leftLedActive)
  {
    digitalWrite(leftLedPin, HIGH);
    digitalWrite(correctLedPin, HIGH);
    delay(500);
    digitalWrite(correctLedPin, LOW);
    digitalWrite(leftLedPin, LOW);
    leftLedActive = false;
  }
  else if (rightLedActive)
  {
    digitalWrite(rightLedPin, HIGH);
    digitalWrite(correctLedPin, HIGH);
    delay(500);
    digitalWrite(correctLedPin, LOW);
    digitalWrite(rightLedPin, LOW);
    rightLedActive = false;
  }
  else if (plusLedActive)
  {
    digitalWrite(plusLedPin, HIGH);
    digitalWrite(correctLedPin, HIGH);
    delay(500);
    digitalWrite(correctLedPin, LOW);
    digitalWrite(plusLedPin, LOW);
    plusLedActive = false;
  }
  else if (minusLedActive)
  {
    digitalWrite(minusLedPin, HIGH);
    digitalWrite(correctLedPin, HIGH);
    delay(500);
    digitalWrite(correctLedPin, LOW);
    digitalWrite(minusLedPin, LOW);
    minusLedActive = false;
  }


  if (proximity_data > 10)
  {
    digitalWrite(wrongLedPin, HIGH);
    //delay(500);
    //digitalWrite(wrongLedPin,LOW);
  }
  else
  {
    digitalWrite(wrongLedPin, LOW);
  }
}

void interruptRoutine()
{
  isr_flag = 1;
}

void handleGesture()
{
  if (apds.isGestureAvailable())
  {
    switch (apds.readGesture())
    {
    case DIR_UP:
      blehid.keyPress('U');
      blehid.keyRelease();

      //Keyboard.write('u');
      Serial.write("u");
      upLedActive = true;
      //lightUpLed();
      break;
    case DIR_DOWN:
      blehid.keyPress('D');
      blehid.keyRelease();

      //Keyboard.write('d');
      Serial.write("d");
      downLedActive = true;
      //lightDownLed();
      break;
    case DIR_LEFT:
      blehid.keyPress('L');
      blehid.keyRelease();

      //Keyboard.write('l');
      Serial.write("l");
      leftLedActive = true;
      //lightLeftLed();
      break;
    case DIR_RIGHT:
      blehid.keyPress('R');
      blehid.keyRelease();

      //Keyboard.write('r');
      Serial.write("r");
      rightLedActive = true;
      //lightRightLed();
      break;
    case DIR_NEAR:
      blehid.keyPress('N');
      blehid.keyRelease();
      plusLedActive = true;
      break;
    case DIR_FAR:
      blehid.keyPress('F');
      blehid.keyRelease();
      minusLedActive = true;
      break;
    default:
      blehid.keyPress('V');
      blehid.keyRelease();
      break;
    }
  }
}


void showHeader()
{
  Serial.println("Gesture Control BlueFruit");
  Serial.print("Based on: iCadeTeensy and Sparkfun APDS9960 tut");
  Serial.print(VERSION);
  Serial.println("shaneshanekavanagh");
}

void showStatus()
{

}

void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}
// End of file.
