#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                     // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <LowPower.h> //energy saving library
#include <DS3232RTC.h>
#include <EEPROM.h>

#define LCD_COLS 20
#define LCD_ROWS 4
#define FACTOR 0.02 //is used for battery level measurement
#define RELAY_PIN 6
#define BLUETOOTH_POWER_PIN 5
#define LCD_BRIGHTNESS_PIN 9
#define BATTERY_LEVEL_PIN A0
#define BUTTON_PIN 4
#define STATE_CHARGING_PIN A6
#define STATE_CHARGED_PIN A7

byte loading, sx =  100, ttable[16];
hd44780_I2Cexp lcd;
float batteryLevel = 0;
int checksum = 0, charging, charged, nowSec;
byte buttonSecond;
boolean buttonIsPressed = false, btIsEnabled = false;
boolean thereIsBell, nextBellIsSoon;
byte minutesTillNextBell, nextBellNum, secondsTillNextBell, i; //"i" is an iterator in "for"
boolean noSleep, sleepMode, wakeLock, sleeping = false, secondTimetable, isAuthorized;
byte tStatus, currentDay = 0, currentSecond = 100;

void introAndBattery();
byte chargingState();

void setup() {
  // put your setup code here, to run once:
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  setSyncInterval(15); //request to RTC every 15 seconds
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BLUETOOTH_POWER_PIN, OUTPUT);
  pinMode(LCD_BRIGHTNESS_PIN, OUTPUT);
  pinMode(STATE_CHARGED_PIN, INPUT);
  pinMode(STATE_CHARGING_PIN, INPUT);
  pinMode(BATTERY_LEVEL_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BLUETOOTH_POWER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  analogWrite(LCD_BRIGHTNESS_PIN, 255);
  Serial.begin(9600); //starting Bluetooth transfer using speed: 9600 bond B/s (the minimal one)
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  batteryLevel = analogRead(BATTERY_LEVEL_PIN);
  introAndBattery();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(batteryLevel);
  lcd.print(" ");
  lcd.print((int)chargingState());
}

void introAndBattery(){ 
  //showing boot animation and checking battery level and state
    lcd.setCursor(0, 0);
    lcd.print("IQBell       v 0.1.1");
    lcd.setCursor(1, 1);
    lcd.print("by Danila Gornushko");
    lcd.setCursor(0, 2);
    lcd.print("Email: dghak@bk.ru");
    lcd.setCursor(3, 3);
    lcd.print("Loading");
    for(i =  0; i < 50; i++){
      batteryLevel -= FACTOR * (batteryLevel - analogRead(BATTERY_LEVEL_PIN));
      lcd.setCursor(11, 3);
      switch ((i/4)%3)
      {
      case 0: lcd.print(".   ");
        break;
      case 1: lcd.print(" .  ");
        break;
      default: lcd.print("  . ");
        break;
      }
      if(i%2 == 0){
        lcd.print(i*2);
        lcd.print("%");
      }
      delay(100);
    }
}

byte chargingState(){
  /*0 - not charging 
  1 - charging
  2 - charged*/
  if((analogRead(STATE_CHARGING_PIN) < 300) && (analogRead(STATE_CHARGED_PIN) < 300)) return 0;
  else if((analogRead(STATE_CHARGING_PIN) > 800) && (analogRead(STATE_CHARGED_PIN) < 800)) return 2;
  else return 1;
}

void loop() {
  // put your main code here, to run repeatedly:

}