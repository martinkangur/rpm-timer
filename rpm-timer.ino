#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//Hardware

const int CLK    = 9; // Pin 9 to clk on encoder
const int DT     = 8; // Pin 8 to DT on encoder
const int btn    = 7; // Pin 7 to btn on encoder
const int relay  = 6; // Pin 6 main relay
const int sensor = 2; // Pin 5 sensor 

//Configuration
const long interval      = 1000; //update interval


//Variables
int AlarmInterval = 10; //when alarm should be triggered in seconds
int relayState  = LOW;
int RotPosition = 0;
int rotation;
int value;
boolean LeftRight;

unsigned long previousMillis = 0;
unsigned long rotationTimer = 0;


U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED


void setup() {
  Serial.begin (9600);
  u8g2.begin();
  pinMode(relay, OUTPUT);
  pinMode (CLK, INPUT_PULLUP);
  pinMode (DT, INPUT_PULLUP);
  pinMode(btn, INPUT_PULLUP);
  pinMode(sensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensor), resetRotationTimer, FALLING);

  rotation = digitalRead(CLK);
  relayState = LOW;
}

void loop() {
  Serial.println(digitalRead(sensor));
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  unsigned long currentMillis = millis();


  digitalWrite(relay, relayState);

  if(digitalRead(btn)==LOW) {
    relayState    = LOW;
    rotationTimer = 0;
  }
  
//  Serial.print(rotationTimer);
//  Serial.print(' ');
//  Serial.println(AlarmInterval);

  value = digitalRead(CLK);
  if (value != rotation) {
    if (digitalRead(DT) != value) {
      RotPosition ++;
      LeftRight = true;
    } else {
      LeftRight = false;
      RotPosition--;
    }
    if (LeftRight) {
      //Serial.println ("clockwise");
      AlarmInterval = AlarmInterval + 5;
    } else {
      //Serial.println("counterclockwise");
      AlarmInterval = AlarmInterval - 5;
    }
    //Serial.print("Encoder RotPosition: ");
    //Serial.println(RotPosition);

  }
  
  rotation = value;

  if (currentMillis - previousMillis >= interval) {
    rotationTimer ++;
    previousMillis = currentMillis;

    if (rotationTimer >= AlarmInterval) {
      relayState = HIGH;
    }
    update_screen();
  }

}

void resetRotationTimer() {
  
  rotationTimer = 0;
}

void update_screen() {
  u8g2.setCursor(0, 10);
  u8g2.print("Timer: ");
  u8g2.print(rotationTimer);

  u8g2.setCursor(0, 20);
  u8g2.print("Alarm interval: ");
  u8g2.print(AlarmInterval);
  u8g2.print("sec");


  u8g2.setCursor(0, 30);
  u8g2.print("Relay: ");
  if (relayState == LOW) {
    u8g2.print("Normal");
  } else {
    u8g2.print("Alarm");
  }

  u8g2.setCursor(0, 40);
  u8g2.print("Button state: ");
  u8g2.print(digitalRead(btn));

  u8g2.sendBuffer();          // transfer internal memory to the display
}
