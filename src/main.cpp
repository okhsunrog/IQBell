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

hd44780_I2Cexp lcd;
float batteryLevel = 0;
boolean sleepMode = false, sleepModeEntering = false, sleepModeExit = false, secondTimetable, wakeLock = false;
byte batteryPercentage, wakeSeconds;
boolean buttonIsPressed = false;
boolean bluetoothSwitch = false, bluetoothState = false;
byte currentBatteryIcon = 255, currentDay = 255, pressedTime, bluetoothOnTime;
byte secondPrev = 255, i, ttable[16]; //"i" is an iterator in "for"

void loading(byte i);
void loadingAndMeasuring();
void introAndBattery();
boolean isCharging();
void chargingMode();
void printBattery(byte level);
void checkBattery();
void updateDisplay();
void checkSleepMode();
void sleep();
void sleepIntro();
void sleepOut();
void onceInSecond();
void checkButton();
void bluetoothPowerControl();
void printBluetoothState();
void printProc(byte x);
void print2digits(byte number);
byte validateWithDefault(byte vl, byte mn, byte mx, byte df);
byte validate(byte vl, byte mn, byte mx);
void setTimetable(boolean isSecond);
boolean isInside(byte startDay, byte startMonth, byte endDay, byte endMonth);

void setup() {
  // put your setup code here, to run once:
  analogReference(INTERNAL);
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
  byte bluetoothCharmap[8] = {
    B00010,
    B10101,
    B01101,
    B00110,
    B01101,
    B10101,
    B00010,
    B0};
  lcd.createChar(1, bluetoothCharmap);
  batteryLevel = analogRead(BATTERY_LEVEL_PIN);
  introAndBattery();
}

void introAndBattery(){ 
  //showing boot animation and checking battery level and state
    lcd.setCursor(0, 0);
    lcd.print("IQBell       v 0.2.0");
    lcd.setCursor(1, 1);
    lcd.print("by Danila Gornushko");
    lcd.setCursor(0, 2);
    lcd.print("Email: dghak@bk.ru");
    loadingAndMeasuring();
}

void loadingAndMeasuring(){
  lcd.setCursor(3, 3);
  lcd.print("Loading");
  for(i =  0; i < 50; i++){
      checkBattery();
      lcd.setCursor(11, 3);
      if(i%2 == 0){
        loading(i/4);
        printProc(i*2);
      } 
      delay(100);
    }
}

void loading(byte i){
  switch (i%3){
      case 0: lcd.print(".   ");
        break;
      case 1: lcd.print(" .  ");
        break;
      default: lcd.print("  . ");
        break;
      }
}

boolean isCharging(){ //true if charging or charged
  return !((analogRead(STATE_CHARGING_PIN) < 50) && (analogRead(STATE_CHARGED_PIN) < 50));
}

void chargingMode(){
  lcd.clear();
  printBluetoothState();
  lcd.setCursor(0, 0);
  if(analogRead(STATE_CHARGED_PIN) < 500){
    lcd.print("Charged!      100% ");
    printBattery(6);
    delay(1000);
  } else{
    lcd.print("Charging");
    checkBattery();
    lcd.setCursor(15, 0);
    printProc(batteryPercentage - 1);
    printBattery(batteryPercentage/15 - 1);
    for(i=0; i < 3; i++){
      lcd.setCursor(8, 0);
      loading(i);
      delay(333);
    }
  }
  checkButton();
  bluetoothPowerControl();
}

void printBattery(byte level) {
  if (level != currentBatteryIcon) { //if battery icon changed then redraw the icon
    byte charmap[8];
    charmap[0] = B01110;
    for (byte i = 1; i < 7; i++) charmap[i] = B10001;
    charmap[7] = B11111;
    for (byte i = 6; i > 6 - level; i--) charmap[i] = B11111;
    lcd.createChar(0, charmap);
    currentBatteryIcon = level;
  }
  lcd.setCursor(19, 0);
  lcd.write(byte(0));
}

void loop() {
  // put your main code here, to run repeatedly:
  if(isCharging()){
    chargingMode();
  } else{
    checkSleepMode();
    if(sleepMode && !wakeLock){
      if(sleepModeEntering){
        sleepIntro();
        sleepModeEntering = false;
      }
      sleep();
    } else {
      checkBattery();
      onceInSecond();
    }
  }
}

void checkBattery(){
  batteryLevel-=FACTOR*(batteryLevel-analogRead(BATTERY_LEVEL_PIN));
  batteryPercentage =  validate(batteryLevel - 252, 0, 100);
}

void updateDisplay(){
  lcd.clear();
  lcd.setCursor(0, 0);
  print2digits(hour());
  lcd.write(':');
  print2digits(minute());
  lcd.write(':');
  print2digits(second());
  lcd.setCursor(0, 1);
  lcd.print(day());
  lcd.write('/');
  lcd.print(month());
  lcd.write('/');
  lcd.print(year());
  lcd.setCursor(11, 0);
  printProc(batteryPercentage);
  printBattery(batteryPercentage/15);
  printBluetoothState();
  if(wakeLock){
    lcd.setCursor(15, 3);
    lcd.print("W: ");
    lcd.print(wakeSeconds);
  }
}

void sleepIntro(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Entering sleep mode");
  lcd.setCursor(0, 3);
  loadingAndMeasuring();
  lcd.clear();
  lcd.noDisplay();
  lcd.noBacklight();
  buttonIsPressed = false;
  bluetoothState = false;
  bluetoothSwitch = false;
  analogWrite(LCD_BRIGHTNESS_PIN, 0);
  Serial.end();
  digitalWrite(BLUETOOTH_POWER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
}

void sleepOut(){
  lcd.display();
  lcd.backlight();
  analogWrite(LCD_BRIGHTNESS_PIN, 255);
  Serial.begin(9600);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Exiting sleep mode");
  loadingAndMeasuring();
}
void sleep(){
  delay(100);
  sleepModeExit = true;
  for(i=0; i<30; i++){
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
    if(digitalRead(BUTTON_PIN) == false) {
      wakeLock = true;
      wakeSeconds = 30;
      sleepOut();
      break;
    }
  }
}

void checkSleepMode(){
  if (currentDay != day()) {
    sleepMode = false;
    if (weekday() == 1 || weekday() == 7) {
      sleepMode = true;
      sleepModeEntering = true;
      Serial.println("Weekend");
    }
    else {
      for (i = 0; i < 8; i++) {
        byte startExceptionMonth = EEPROM.read(32 + i * 4);
        byte startExceptionDay = EEPROM.read(33 + i * 4);
        byte endExceptionMonth = EEPROM.read(34 + i * 4);
        byte endExceptionDay = EEPROM.read(35 + i * 4);
        if(startExceptionDay > 31 || endExceptionDay > 31 || 
        startExceptionMonth > 12 || endExceptionMonth > 12 || startExceptionDay == 0 || 
        startExceptionMonth == 0 || endExceptionDay == 0 || endExceptionMonth == 0) continue;
        if (isInside(startExceptionDay, startExceptionMonth, endExceptionDay, endExceptionMonth)){
          sleepMode = true;
          sleepModeEntering = true;
          Serial.println("Long");
          break;
        }
      }
      secondTimetable = false;
      if(!sleepMode){
        for (i = 0; i < 8; i++) {
          byte exceptionMonth = EEPROM.read(64 + i * 2);
          byte exceptionDay = EEPROM.read(65 + i * 2);
          boolean shDay;
          shDay = (exceptionMonth > 127);
          exceptionMonth &= B01111111;
          if (exceptionMonth == month() && exceptionDay == day()) {
            if (shDay == false){
               sleepMode = true;
               sleepModeEntering = true;
               Serial.println("Short");
            } else secondTimetable = true;
          }
        }
      }
    }
    setTimetable(secondTimetable);
    currentDay = day();
  }
}

boolean isInside(byte startDay, byte startMonth, byte endDay, byte endMonth) {
  int today = (month() - 1) * 31 + (day() - 1);
  int startDate = (startMonth - 1) * 31 + (startDay - 1);
  int endDate = (endMonth - 1) * 31 + (endDay - 1);
  if (startDate > endDate) {
    if (today < 186) {
      if (today < startDate) return true;
      else return false;
    }
    else {
      if (today > endDate) return true;
      else return false;
    }
  }
  else {
    if (today > startDate && today < endDate) return true;
    else return false;
  }
}

void onceInSecond(){
  if(second()!=secondPrev){
    if(wakeLock){
      wakeSeconds--;
      if(wakeSeconds == 0){
        wakeLock = false;
        sleepIntro();
        return;
      }
    }
    updateDisplay();
    checkButton();
    bluetoothPowerControl();
    secondPrev = second();
  }
}

void setTimetable(boolean isSecond) {
  for (i = 0; i < 16; i++)
    ttable[i] = EEPROM.read(isSecond ? i + 16 : i);
}


// Ensures that given number is inside [min;max] range
byte validateWithDefault(byte vl, byte mn, byte mx, byte df) {
  if (vl < mn || vl > mx) return df;
  return vl;
}

byte validate(byte vl, byte mn, byte mx) {
  if (vl < mn) return mn;
  if (vl > mx) return mx;
  return vl;
}

void printProc(byte x) {
  lcd.print(x);
  lcd.print("% ");
}

void print2digits(byte number){
  if (number < 10) lcd.print('0');
  lcd.print(number);
}

void checkButton(){
  if(!digitalRead(BUTTON_PIN) && !buttonIsPressed){
    buttonIsPressed = true;
    pressedTime = 0;
    wakeSeconds = 30;
  }
  if(!digitalRead(BUTTON_PIN) && buttonIsPressed){
    pressedTime++;
    if(pressedTime > 5){
      bluetoothSwitch = true;
    } 
  }
  if(digitalRead(BUTTON_PIN) && buttonIsPressed){
    buttonIsPressed=false;
  }
}

void printBluetoothState(){
  if(bluetoothState){
    Serial.println("Print BT char");
    lcd.setCursor(19, 1);
    lcd.write(byte(1));
    lcd.setCursor(16, 1);
    lcd.print(bluetoothOnTime);
  }
}

void bluetoothPowerControl(){
  if(bluetoothSwitch && !bluetoothState){
    digitalWrite(BLUETOOTH_POWER_PIN, HIGH);
    bluetoothOnTime = 30;
    bluetoothSwitch = false;
    bluetoothState = true;
  }
  else if(bluetoothSwitch && bluetoothState){
    bluetoothOnTime = 30;
    bluetoothSwitch = false;
  }
  else if(!bluetoothSwitch && bluetoothState){
    if(bluetoothOnTime == 0){
      bluetoothState = false;
      digitalWrite(BLUETOOTH_POWER_PIN, LOW);
    } else bluetoothOnTime--;
  }
}
