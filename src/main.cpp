#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                     // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <LowPower.h> //energy saving library
#include <DS3232RTC.h>
#include <EEPROM.h>
#include <CRC32.h>

#define LCD_COLS 20
#define LCD_ROWS 4
#define FACTOR 0.0001 //is used for battery level measurement
#define RELAY_PIN 6
#define BLUETOOTH_POWER_PIN 5
#define LCD_BRIGHTNESS_PIN 9
#define BATTERY_LEVEL_PIN A0
#define BUTTON_PIN 4
#define STATE_CHARGING_PIN A6
#define STATE_CHARGED_PIN A7
#define BLUETOOTH_ON_TIME 15
#define BELL_ON_TIME 7
#define WAKE_LOCK_TIME 30
#define BUTTON_TIME 3

typedef union {
	uint32_t lValue;
	uint8_t bValue[sizeof(lValue)];
} ULongByBytes; //is using for sending and receiving unsigned long via Serial

ULongByBytes checksum;
hd44780_I2Cexp lcd;
CRC32 crc;
float batteryLevel = 0, correctBatLevel, tempBL;
boolean sleeping = false, secondTimetable, wakeLock = true, newMeasuringFlag = false;
byte batPercentage, wakeSeconds = WAKE_LOCK_TIME, displayBrightness;
boolean buttonIsPressed = false, isAuthorized, activeConnection = false;
boolean bluetoothSwitch = false, bluetoothIsOn = false, isHoliday = false;
boolean bellIsOn = false, measuring = false;
byte currentBatteryIcon = 255, currentDay = 255, pressedTime, bluetoothOnTime, mode = 0, prevMode = 0;
byte secondPrev = 255, i, ttable[16], prevBellNum = 255, relayOnTime = 0; //"i" is an iterator in "for"
byte firstBell, lastBell, prevBell, numOfBell, ringingState = 0, temperature = 0;
int firstBellMinute, lastBellMinute, ii;
long timeTillBell;
boolean workshop[15] = {false, true, true, true,  true, false, true,  true, true, true, false, true, true, true, true};
boolean assembly[12] = {false, true, true, false, true, true, false, true, true,  false, true, true};

void loading(byte i);
void loadingAndMeasuring();
void intro();
void printBattery(byte level);
void checkBattery();
void checkBluetooth();
void printWeekDay();
void updateDisplay();
void printTemperature();
void checkMode();
void clearInput();
void getData();
void sleep();
void sleepIntro();
void sleepOut();
void onceInSecond();
void checkButton();
void bluetoothPowerControl();
void printBluetoothState();
void printProc(byte x);
void print2digits(byte number);
byte validate(byte vl, byte mn, byte mx);
boolean isInside(byte startDay, byte startMonth, byte endDay, byte endMonth);
byte timeIsSet();
void waitForInput();
void sendByte(byte data);
byte getByte(byte numOfBytes = 1);
boolean Checksum();
void getData();
void sendingInfo();
void sendingExtraInfo();
void settingLongExceptions();
void settingShortExceptions();
void settingTime();
void sendChecksum();
void settingTimetable(boolean isSecond);
void printWakeLockState();
void setDisplayBrightness();
void timeTick();
void bellControl();
void printBellTime();
void manualBell(byte type);
void printTimeAndDate();
void printMode();
void printTimetable();
void printRelayState();

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
	displayBrightness = EEPROM.read(88);
	analogWrite(LCD_BRIGHTNESS_PIN, displayBrightness);
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
	byte bellCharmap[8] = {
		B0,
		B00100,
		B01010,
		B01010,
		B01010,
		B11111,
		B00100,
		B0};
	lcd.createChar(2, bellCharmap);
	batteryLevel = analogRead(BATTERY_LEVEL_PIN);
	intro();
}

void intro(){ //showing boot animation
	lcd.setCursor(0, 0);
	lcd.print("IQBell  Smart School");
	lcd.setCursor(1, 1);
	lcd.print("by Danila Gornushko");
	lcd.setCursor(0, 2);
	lcd.print("Email: dghak@bk.ru");
	loadingAndMeasuring();
}

void loadingAndMeasuring(){
	lcd.setCursor(3, 3);
	lcd.print("Loading");
	measuring = true;
	for(i = 0; i < 25; i++){
		lcd.setCursor(11, 3);
		loading(i/4);
		printProc(i*4);
		for(ii = 0; ii < 1200; ii++) checkBattery();
	}
	measuring = false;
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
	if(!bluetoothIsOn && !bellIsOn){
		if(newMeasuringFlag){
			lcd.clear();
			lcd.setCursor(4, 0);
			lcd.print("Measuring");
			loadingAndMeasuring();
			newMeasuringFlag = false;
		}else checkBattery();
	}
	checkMode();
	if(mode == 0) wakeLock = false;
		if(mode > 0 && !wakeLock){
			if(!sleeping) sleepIntro();
			sleep();
		} else if(sleeping) sleepOut();
	onceInSecond();
	if(bluetoothIsOn) checkBluetooth();
	if(batPercentage > 20) analogWrite(LCD_BRIGHTNESS_PIN, displayBrightness);
	else if(batPercentage > 10) analogWrite(LCD_BRIGHTNESS_PIN, displayBrightness/2);
	else analogWrite(LCD_BRIGHTNESS_PIN, 7);
}

void checkBattery(){
	if(mode == 5) batPercentage = 100;
	else{
		int level = analogRead(BATTERY_LEVEL_PIN);
		if(mode == 4) level -= 15;
		batteryLevel-=FACTOR*(batteryLevel-level);
		tempBL = batteryLevel - 252;
		if(tempBL > 50) correctBatLevel = ((tempBL)*1.92) - 92;
		else correctBatLevel = tempBL/10;
		if(mode == 4) batPercentage = validate(correctBatLevel, 0, 99);
		else{
			byte tempLevel = validate(correctBatLevel, 0, 100);
			if(measuring) batPercentage = tempLevel;
			else if(tempLevel < batPercentage) batPercentage = tempLevel;
		}
	}
}

void updateDisplay(){
	lcd.clear();
	printTimeAndDate();
	printWeekDay();
	lcd.setCursor(15, 0);
	printProc(batPercentage);
	printBattery(batPercentage/15);
	printBluetoothState();
	printRelayState();
	printWakeLockState();
	printTemperature();
	printBellTime();
	printMode();
}

void printMode(){
	lcd.setCursor(0, 3);
	switch (mode){
	case 0:
		lcd.print("Classes ");
		printTimetable();		
		break;
	case 1:
		lcd.print("Holiday");
		break;
	case 2:
		lcd.print("Not started ");
		printTimetable();		
		break;
	case 3:
		lcd.print("Finished ");
		printTimetable();
		break;
	case 4:
		lcd.print("Charging");
		break;
	default:
		lcd.print("Charged");
		break;
	}
}

void printTimetable(){
	if(secondTimetable) lcd.print("Short");
	else lcd.print("Main");
}

void printBellTime(){
	if(mode == 0 && numOfBell != 255){
		lcd.setCursor(11, 1);
		print2digits(byte(timeTillBell/60));
		lcd.print(":");
		print2digits(byte(timeTillBell%60));
		lcd.setCursor(11, 2);
		print2digits(ttable[numOfBell]/12+8);
		lcd.print(":");
		print2digits(ttable[numOfBell]%12*5);
		lcd.setCursor(14, 3);
		print2digits(numOfBell+1);
	}
}

void printTimeAndDate(){
	lcd.setCursor(0, 0);
	print2digits(hour());
	lcd.write(':');
	print2digits(minute());
	lcd.write(':');
	print2digits(second());
	lcd.setCursor(0, 1);
	print2digits(day());
	lcd.write('/');
	print2digits(month());
	lcd.write('/');
	lcd.print(year());
}

void printTemperature(){
	lcd.setCursor(9, 0);
	temperature = RTC.temperature() / 4;
	lcd.print(temperature);
	lcd.write(byte(0xDF));
	lcd.print("C");
}

void printRelayState(){
	if(ringingState > 0){
		lcd.setCursor(19, 1);
		lcd.write(byte(2));
		if(relayOnTime != 0){
			lcd.setCursor(17, 1);
			print2digits(relayOnTime);
		}
	}
}

void printWakeLockState(){
	if(wakeLock){
		lcd.setCursor(19, 2);
		lcd.print("W");
		if(wakeSeconds < WAKE_LOCK_TIME + BLUETOOTH_ON_TIME - 2){
			lcd.setCursor(17, 2);
 			print2digits(wakeSeconds);
		}
	}
}

void printBluetoothState(){
	if(bluetoothIsOn){
		lcd.setCursor(19, 3);
		lcd.write(byte(1));
		if(bluetoothOnTime < BLUETOOTH_ON_TIME - 2){
			lcd.setCursor(17, 3);
			print2digits(bluetoothOnTime);
		} 
	}
}

void sleepIntro(){
	lcd.clear();
	lcd.noDisplay();
	lcd.noBacklight();
	buttonIsPressed = false;
	bluetoothIsOn = false;
	bluetoothSwitch = false;
	analogWrite(LCD_BRIGHTNESS_PIN, 0);
	Serial.end();
	digitalWrite(BLUETOOTH_POWER_PIN, LOW);
	digitalWrite(RELAY_PIN, LOW);
}

void sleepOut(){
	sleeping = false;
	lcd.display();
	lcd.backlight();
	analogWrite(LCD_BRIGHTNESS_PIN, displayBrightness);
	Serial.begin(9600);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Exiting sleep mode");
	loadingAndMeasuring();
}

void sleep(){
	delay(100);
	sleeping = true;
	secondPrev = 255;
	for(i=0; i<15; i++){
		LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
		if(digitalRead(BUTTON_PIN) == false){
			wakeLock = true;
			wakeSeconds = WAKE_LOCK_TIME;
			break;
		}
	}
	setSyncProvider(RTC.get);   // we need to sync time after sleep mode
	setSyncInterval(15);
}

void checkMode(){
	mode = 0; //average mode
  	if (currentDay != day()) {
		isHoliday = false;
		if (weekday() == 1 || weekday() == 7) isHoliday = true;
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
					isHoliday = true;
					break;
				}
			}
			secondTimetable = false;
			if(!isHoliday){
				for (i = 0; i < 8; i++) {
					byte exceptionMonth = EEPROM.read(64 + i * 2);
					byte exceptionDay = EEPROM.read(65 + i * 2);
					boolean shortDay;
					shortDay = (exceptionMonth > 127);
					exceptionMonth &= B01111111;
					if (exceptionMonth == month() && exceptionDay == day()) {
						if (shortDay == false) isHoliday = true;
						else secondTimetable = true;
					}
				}
			}
		}
		for (i = 0; i < 16; i++) ttable[i] = EEPROM.read(secondTimetable ? i + 16 : i);
		firstBell = 255;
		lastBell = 0;
		for(i = 0; i< 16; i++){
			if(ttable[i] > 127) continue;
			if(ttable[i] > lastBell) lastBell = ttable[i];
			if(ttable[i] < firstBell) firstBell = ttable[i];
		}
		firstBellMinute = (firstBell/12+8)*60 + (firstBell%12*5);
		lastBellMinute = (lastBell/12+8)*60 + (lastBell%12*5);
		currentDay = day();
	}
	if(analogRead(STATE_CHARGED_PIN) > 1000) mode = 4;
	else if(analogRead(STATE_CHARGING_PIN) > 1000) mode = 5;
	if(mode == 0){
		if(isHoliday) mode = 1; //holiday mode
		else if((firstBellMinute - 10) >= (hour()*60+minute())) mode = 2; //classes have not started
		else if((lastBellMinute + 10) <= (hour()*60+minute())) mode = 3; //classes have finished
	}
	if(mode != prevMode){
		wakeLock = true;
		wakeSeconds = WAKE_LOCK_TIME;
	}
	prevMode = mode;
}

boolean isInside(byte startDay, byte startMonth, byte endDay, byte endMonth) {
	int today = (month() - 1) * 31 + (day() - 1);
	int startDate = (startMonth - 1) * 31 + (startDay - 1);
	int endDate = (endMonth - 1) * 31 + (endDay - 1);
	if(startDate == endDate) return (startDate == today);
	else if (startDate > endDate) {
		if (today < 186) {
			if (today <= endDate) return true;
			else return false;
		}
		else {
			if (today >= startDate) return true;
			else return false;
		}
	}
	else {
		if (today >= startDate && today <= endDate) return true;
		else return false;
	}
}

void onceInSecond(){
	if(second()!=secondPrev){
		if(mode == 0) timeTick();
		if(mode == 0 || wakeLock){
			updateDisplay();
			checkButton();
			bluetoothPowerControl();
			if(mode == 0) bellControl();
			else ringingState = 0;
			secondPrev = second();
		}
		if(wakeLock){
			if(wakeSeconds == 0){
			wakeLock = false;
			} else if(bluetoothIsOn) wakeSeconds = bluetoothOnTime + WAKE_LOCK_TIME;
			else wakeSeconds--;
		}
	}
}

void timeTick(){
	prevBell = 255;
	numOfBell = 255;
	for(i=0; i<16; i++){
		if(ttable[i] > 127) continue;
		if(ttable[i] < prevBell){
			if((((ttable[i]/12+8)*3600+(ttable[i]%12*5)*60) - (hour()*3600 + minute()*60 + second())) >= 0){
				prevBell = ttable[i];
				numOfBell = i;
			}
		}
	}
	timeTillBell = (((ttable[numOfBell]/12+8)*3600+(ttable[numOfBell]%12*5)*60) - (hour()*3600 + minute()*60 + second()));
	if(timeTillBell == 0) ringingState = 1;
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
		if(bluetoothIsOn) bluetoothOnTime = BLUETOOTH_ON_TIME;
		else wakeSeconds = WAKE_LOCK_TIME;
	}
	if(!digitalRead(BUTTON_PIN) && buttonIsPressed){
		pressedTime++;
		if(pressedTime >= BUTTON_TIME){
			bluetoothSwitch = true;
		}
	}
	if(digitalRead(BUTTON_PIN) && buttonIsPressed){
		buttonIsPressed=false;
	}
}

void bluetoothPowerControl(){
	if(batPercentage < 10) bluetoothSwitch = false;
	if(bluetoothSwitch && !bluetoothIsOn){
		digitalWrite(BLUETOOTH_POWER_PIN, HIGH);
		bluetoothOnTime = BLUETOOTH_ON_TIME;
		bluetoothSwitch = false;
		bluetoothIsOn = true;
	}
	else if(bluetoothSwitch && bluetoothIsOn){
		bluetoothOnTime = BLUETOOTH_ON_TIME;
		bluetoothSwitch = false;
	}
	else if(!bluetoothSwitch && bluetoothIsOn){
		if(bluetoothOnTime == 0 || batPercentage < 10){
			bluetoothIsOn = false;
			digitalWrite(BLUETOOTH_POWER_PIN, LOW);
		} else if(activeConnection){
			activeConnection = false;
			bluetoothOnTime = BLUETOOTH_ON_TIME;
		} else bluetoothOnTime--;
	}
}

void bellControl(){
	switch (ringingState){
	case 1:
		if(!bellIsOn){
			digitalWrite(RELAY_PIN, HIGH);
			relayOnTime = BELL_ON_TIME;
			bellIsOn = true;
		} else{
			if(relayOnTime == 0){
				bellIsOn = false;
				ringingState = 0;
				digitalWrite(RELAY_PIN, LOW);
			} else relayOnTime--;
		}
		break;
	case 2:
		if(!bellIsOn){
			bellIsOn = true;
			relayOnTime = 15;
		}
		if(bellIsOn){
			relayOnTime--;
			digitalWrite(RELAY_PIN, workshop[relayOnTime]);
			if(relayOnTime == 0){
				bellIsOn = false;
				ringingState = 0;
			}
		}
		break;
	case 3:
		if(!bellIsOn){
			bellIsOn = true;
			relayOnTime = 12;
		}
		if(bellIsOn){
			relayOnTime--;
			digitalWrite(RELAY_PIN, assembly[relayOnTime]);
			if(relayOnTime == 0){
				bellIsOn = false;
				ringingState = 0;
			}
		}
		break;
	}
}

void printWeekDay(){
	lcd.setCursor(0, 2);
	switch (weekday())
	{
	case 1: lcd.print("Sunday");
		break;
	case 2: lcd.print("Monday");
		break;
	case 3: lcd.print("Tuesday");
		break;
	case 4: lcd.print("Wednesday");
		break;
	case 5: lcd.print("Thursday");
		break;
	case 6: lcd.print("Friday");
		break;
	case 7: lcd.print("Saturday");
		break;
	}
}

void checkBluetooth(){
	if(Serial.available()){
		delay(50);
		if(Serial.available()>7){
			isAuthorized = true;
			for (i = 0; i < 8; i++) if(Serial.read() != EEPROM.read(80 + i)) isAuthorized = false;
			if (isAuthorized) {
				activeConnection = true;
				clearInput();
				Serial.write(11); //OK message
				delay(100);
				if (Serial.available() > 0)  getData();
			}
		}
		clearInput();
	}
}

void clearInput(){
	while(Serial.available()) Serial.read();
}

byte timeIsSet(){
	return timeIsSet() == timeSet ? 1 : 0;
}

void waitForInput(){
	while (Serial.available() == 0);
	delay(20);
}

void sendByte(byte data){
	Serial.write(data);
	crc.update(data);
}

byte getByte(byte numOfBytes = 1){
	byte data;
	while (numOfBytes) {
		data = Serial.read();
		crc.update(data);
		numOfBytes--;
	}
	return data;
}

void sendChecksum(){
	checksum.lValue = crc.finalize();
	Serial.write(checksum.bValue, 4);
}

boolean Checksum(){
	for(i=0; i<4; i++){
		checksum.bValue[i] = byte(Serial.read());
	}
	return (crc.finalize() == checksum.lValue);
}

void getData(){
	crc.reset();
	byte command = getByte();
	switch (command) {
		case 0: sendingInfo();
			break;
		case 1: settingTimetable(false);
			break;
		case 2: settingLongExceptions();
			break;
		case 3: settingShortExceptions();
			break;
		case 4: settingTime();
			break;
		case 5: sendingExtraInfo();
			break;
		case 6: settingTimetable(true);
			break;
		case 7: setDisplayBrightness();
			break;
		case 8: manualBell(1);
			break;
		case 9: manualBell(2);
			break;
		case 10: manualBell(3);
			break;
	}
	currentDay = 255;
	clearInput();
}

void manualBell(byte type){
	if(Checksum()){
		Serial.write(11);
		if(ringingState == 0 && mode == 0) ringingState = type;
	}
	else Serial.write(35);
}

void setDisplayBrightness(){
	byte tempBrightness = getByte();
	if (Checksum()) {
		Serial.write(11);
		displayBrightness = tempBrightness;
		EEPROM.write(88, displayBrightness);
		newMeasuringFlag = true;
	}
	else Serial.write(35);
}

void sendingInfo(){
	getByte(4); //ignoring 4 bytes. However, they are added to the checksum
	if (Checksum()) {
		crc.reset();
		ULongByBytes toSend;
		sendByte(11);
		toSend.lValue = now();
		for (i = 0; i < 4; i++) sendByte(toSend.bValue[i]); //sending current time in unix format as 4 bytes
		byte modeToSend = mode;
		if(secondTimetable) modeToSend += 128;
		sendByte(modeToSend);
		sendByte(temperature);
		sendByte(ringingState);
		sendByte(relayOnTime);
		sendByte(numOfBell);
		sendChecksum();
	}
	else Serial.write(35); //Failed
}

void sendingExtraInfo(){
	getByte(4); //ignoring 4 bytes. However, they are added to the checksum
	if (Checksum()) {
		crc.reset();		
		sendByte(11);
		for(i=0; i<80; i++) sendByte(EEPROM.read(i)); //sending timetables' and exceptions' data
		sendByte(displayBrightness);
		sendByte(batPercentage);
		sendChecksum();
	}
	else Serial.write(35); //Failed
}

void settingLongExceptions(){
	byte tempArr[32];
	for (i = 0; i < 32; i++) tempArr[i] = getByte();
	if (Checksum()) {
		Serial.write(11);
		for (i = 0; i < 32; i++) EEPROM.write(32 + i, tempArr[i]);
	}
	else Serial.write(35);
}

void settingShortExceptions(){
	byte tempArr[16];
	for (i = 0; i < 16; i++) tempArr[i] = getByte();
	if (Checksum()) {
		Serial.write(11);
		for (i = 0; i < 16; i++) EEPROM.write(64 + i, tempArr[i]);
	}
	else Serial.write(35);
}

void settingTime(){
	ULongByBytes val;
	for (i = 0; i < 4; i++) val.bValue[i] = getByte(); //receiving 4 bytes
	if (Checksum()) {
		Serial.write(11);
		RTC.set(val.lValue);   // set the RTC and the system time to the received value
		setTime(val.lValue);  // unix-format time is used here as well
	}
	else Serial.write(35);
}

void settingTimetable(boolean isSecond){
	byte tempArr[16];
	byte k;
	if(isSecond) k = 16;
	else k = 0;
	for (i = 0; i < 16; i++) tempArr[i] = getByte();
	if (Checksum()) {
		Serial.write(11);
		for (i = 0; i < 16; i++) EEPROM.write(i+k, tempArr[i]);
	}
	else Serial.write(35);
}
