/****************************************************************************

  RoastLoggerMax6675.ino 
  
  This sketch is for use with MAX 6675 thermocouple interface chips. A separate sketch 
  is available for MAX 31855 chips.

  See the "Contributed Libraries" section of http://www.arduino.cc/en/Reference/Libraries
  for details of how to install it.
  
 ****************************************************************************/
 // Revision history: - of RoastLoggerMax6675
 //  20120201:  Version 1.2 - Made available for download from downloads page
 //  20120303:  Version 1.3 - Added #define CELSIUS as default, user to comment out for Fahrenheit output
 //  20120324:  Version 1.4 - Serial output limited to 1 decimal place

/**************************************************************************
 
  This sketch verifies/compiles on the Arduino IDE version 1.0
  
  Previous versions of this sketch written and tested on Arduino
  IDE versions 0018 and 0022 are also available
  
  The Arduino IDE version 1.0 has significant changes to previous versions of the IDE
  that have necessitated changes to this sketch and the libraries it uses.  The modified
  libraries are distributed with this sketch.
   
  This sketch assumes that you are reading from two thermocouples via two Max 6675 chips.
  If you only wish to use one thermocouple via one chip comment out the lines indicated
  in the doSerialOutput method to prevent indicating an error reading (-1) on t2.
 
 ****************************************************************************/

// ------------------------------------------------------------------------------------------
// Copyright (c) 2011-2012, Tom Coxon (GreenBean TMC)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of 
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list 
//   of conditions and the following disclaimer in the documentation and/or other materials 
//   provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

/****************************************************************************
 * 
 * Arduino control of a Hottop roaster by T R Coxon
 * 
 * Using an Arduino Duemilanove 328 on Hottop P and B Roasters
 *
 * Sends data and receives instructions as key=value pairs.                 
 * Sends temperatures T1 and T2 at sendTimePeriod intervals as "T1=123.6" 
 * Accepts instructions for power level, set temperature etc.
 * Example: Send to Arduino "power=80"  or "setT=240.6"  
 ****************************************************************************/

/****************************************************************************
 * 
 * Connections to Arduino and Behmor for heater power control:
 * 
 * Please note that the following describes the modifications to my Behmor
 * 1600+.  You should not attempt this unless you are suitably qualified /
 * experienced.  Modifying your roaster will probably void your warranty
 * and may result in damage or injury. You do this at your own risk.
 *
 * http://blog.soemarko.com/post/109402505018/behmor-mod-phase-2-heat-control-with-arduino
 * 
 ****************************************************************************/

/****************************************************************************
 * 
 * Control options.
 * 
 * 1. Manual as original by closing switch
 * 2. Arduino control by using a potentiometer
 * 3. Computer control by sending commands to arduino
 * 
 ****************************************************************************/

/****************************************************************************
 * 
 * Modified by Soemarko Ridwan
 * 25 Jan 2015
 * for modded Behmor 1600+
 * 
 ****************************************************************************/

/****************************************************************************
 *
 * Change log.
 *
 * 31 Jan 2015:
 * - Remove PID dependencies
 * - Add support for a potentio meter (5V, Pin A0)
 * - Add I2C LCD
 * - Add support for Artisan
 *
 ****************************************************************************/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

#define CELSIUS //RoastLogger default is Celsius - To output data in Fahrenheit comment out this one line 
#define DP 1  // No. decimal places for serial output

#define maxLength 30                  // maximum length for strings used

const int arduino = 1;                // controlled via digitalRead on ctrlPin
const int computer = 0;

/****************************************************************************
 * 
 * Arduino pin assignments
 * 
 * The following are the pin assignments used by my setup.
 * 
 * You only need to change the following 5 constants to suit the pin
 * assignments used by your setup.  
 * 
 ****************************************************************************/

bool isArtisan = false;

// set pin numbers:
const int pwmPin =  9;         // digital pin for pulse width modulation of heater

// thermocouple reading Max 6675 pins
const int SO  = 2;		// SO pin on MAX6675
const int SCKa = 3;		// SCKa pin on MAX6675
const int CS1 = 6;		// CS (chip 1 select) pin on MAX6675
const int CS2 = 5;		// CS (chip 2 select)  pin on MAX6675

// pots
const int potPin = A0;
const int potMax = 1020;
int potVal = 0;
float smoothedValue = 0;
float lastReadValue = 0;
float alpha = 0.5;

// LM35
const int lmPin = A1;
float lmTemp = 0.0;

// LCD
byte pcChar[8] = {
	0b11111,
	0b00101,
	0b00101,
	0b00010,
	0b00000,
	0b01110,
	0b10001,
	0b10001
};
byte ardChar[8] = {
	0b01110,
	0b10001,
	0b01010,
	0b00100,
	0b01010,
	0b10001,
	0b01110,
        0b00000
};
LiquidCrystal_I2C lcd(0x27,16,2);

const int ctrlPin = 12;
const int ssrCtrlPin = 11;

/****************************************************************************
 *  After setting the above pin assignments you can use the remainder of this
 *   sketch as is or change it, if you wish, to add additional functionality
 * 
 ****************************************************************************/


// time constants
const int timePeriod = 2000;          // total time period of PWM milliseconds see note on setupPWM before changing
const int tcTimePeriod = 250;         // 250 ms loop to read thermocouples

// thermocouple settings
float calibrate1 = 0.0;	// Temperature compensation for T1
float calibrate2 = 0.0;	// Temperature compensation for T2

// set global variables

//temporary values for temperature to be read
float temp1 = 0.0;                   // temporary temperature variable
float temp2 = 0.0;                   // temporary temperature variable 
float t1 = 0.0;                      // Last average temperature on thermocouple 1 - average of four readings
float t2 = 0.0;                      // Last average temperature on thermocouple 2 - average of four readings
float tCumulative1 = 0.0;            // cumulative total of temperatures read before averaged
float tCumulative2 = 0.0;            // cumulative total of temperatures read before averaged
int noGoodReadings1 = 0;             // counter of no. of good readings for average calculation 
int noGoodReadings2 = 0;             // counter of no. of good readings for average calculation

int inByte = 0;                       // incoming serial byte
String inString = String(maxLength);  // input String


// loop control variables
unsigned long lastTCTimerLoop;        // for timing the thermocouple loop
int tcLoopCount = 0;                  // counter to run serial output once every 4 loops of 250 ms t/c loop

// PWM variables
int  timeOn;                          // millis PWM is on out of total of timePeriod (timeOn = timePeriod for 100% power)
unsigned long lastTimePeriod;         // millis since last turned on pwm
int power = 100;                      //use as %, 100 is always on, 0 always off default 100
int currentMotor = 0;

int controlBy = arduino;              // default is arduino control. PC sends "pccontrol" to gain control or
                                      // swapped back to Arduino control if PC sends "arduinocontrol"

void setup()
{
  // start serial port at 115200 baud:
  Serial.begin(115200);

  // use establish contact if you want to wait until 'A' sent to Arduino before start - not used in this version
  // establishContact();  // send a byte to establish contact until receiver responds 
  setupPWM();
  
  //set up pin modes for Max6675's
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(SCKa, OUTPUT);
  // deselect both Max6675's
  digitalWrite(CS1, HIGH);
  digitalWrite(CS2, HIGH);
  
  Serial.println("Reset");            // flag that Arduino has reset used for debugging
  
  pinMode(13, OUTPUT); digitalWrite(13, LOW); // force LCD to be off
  
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, ardChar);
  lcd.createChar(1, pcChar);

  pinMode(ctrlPin, INPUT);
  pinMode(ssrCtrlPin, INPUT);
}

/****************************************************************************
 * Set up power pwm control for heater.  Hottop uses a triac that switches
 * only on zero crossing so normal pwm will not work.
 * Minimum time slice is a half cycle or 10 millisecs in UK.  
 * Loop in this prog may need up to 10 ms to complete.
 * I will use 20 millisecs as time slice with 100 power levels that
 * gives 2000 milliseconds total time period.
 * The Hottop heater is very slow responding so 2 sec period is not a problem.
 ****************************************************************************/
void setupPWM() {
  // set the digital pin as output:
  pinMode(pwmPin, OUTPUT);      

  //setup PWM
  lastTimePeriod = millis();
  digitalWrite(pwmPin, HIGH);//set PWM pin on to start
}

// this not currently used
void establishContact() {
  //  while (Serial.available() <= 0) {
  //    Serial.print('A', BYTE);   // send a capital A
  //    delay(300);
  //  }
  Serial.println("Opened but no contact yet - send A to start");
  int contact = 0;
  while (contact == 0){
    if (Serial.available() > 0) {
      inByte = Serial.read();
      Serial.println(inByte);
      if (inByte == 'A'){ 
        contact = 1;
        Serial.println("Contact established starting to send data");
      }    
    }
  }  
}

/****************************************************************************
 * Toggles the heater on/off based on the current power level.  Power level
 * may be determined by arduino or computer.
 ****************************************************************************/
void doPWM()
{
  timeOn = timePeriod * power / 100; //recalc the millisecs on to get this power level, user may have changed
 
 if (millis() - lastTimePeriod > timePeriod) lastTimePeriod = millis();
 if (millis() - lastTimePeriod < timeOn && digitalRead(ssrCtrlPin)){
      digitalWrite(pwmPin, HIGH); // turn on
  } else {
      digitalWrite(pwmPin, LOW); // turn off
 }
 
}

/****************************************************************************
 * Called to set power level. Now always safe as hardware only turns heater on
 * if BOTH the Arduino AND the Hottop control board call for heat.
 ****************************************************************************/
void setPowerLevel(int p)
{
  if (p > -1 && p < 101) power = p;
}

/****************************************************************************
 * Called when an input string is received from computer
 * designed for key=value pairs or simple text commands. 
 * Performs commands and splits key and value 
 * and if key is defined sets value otherwise ignores
 ****************************************************************************/
void doInputCommand()
{
  float v = -1;
  inString.toLowerCase();
  int indx = inString.indexOf('=');

  if (indx < 0) {  //this is a message not a key value pair

    /*if (inString.equals("pccontrol")) {
      controlBy = computer;      
    } 
    else if (inString.equals("arduinocontrol")) {
      controlBy = arduino;        
    }*/

    if(inString.equals("chan;1200")) {
	isArtisan = true;
	Serial.println("# Active channels set to 2");
    }
    else if(inString.equals("read")) {
	String x = String(lmTemp) + "," + String(t2) + "," + String(t1);
	Serial.println(x);
    }
    Serial.flush();

  } 
  else {  //this is a key value pair for decoding
    String key = inString.substring(0, indx);
    String value = inString.substring(indx+1, inString.length());

    //parse string value and return float v
    char buf[value.length()+1];
    value.toCharArray(buf,value.length()+1);
    v = atof (buf); 
 
    //only set value if we have a valid positive number - atof will return 0.0 if invalid
    if (v >= 0)
    {
      if (key.equals("power")  && controlBy == computer && v < 101){  
        
        setPowerLevel((long) v);//convert v to integer for power 
                
      }
      else if (key.equals("fan") && controlBy == computer && v < 101) {
        // motor control
        currentMotor = (long)v;
      }
    }
  }
}

/****************************************************************************
 * check if serial input is waiting if so add it to inString.  
 * Instructions are terminated with \n \r or 'z' 
 * If this is the end of input line then call doInputCommand to act on it.
 ****************************************************************************/
void getSerialInput()
{
  //check if data is coming in if so deal with it
  if (Serial.available() > 0) {

    // read the incoming data as a char:
    char inChar = Serial.read();
    // if it's a newline or return or z, print the string:
    if ((inChar == '\n') || (inChar == '\r') || (inChar == 'z')) {

      //do whatever is commanded by the input string
      if (inString.length() > 0) doInputCommand();
      inString = "";        //reset for next line of input
    } 
    else {
      // if we are not at the end of the string, append the incoming character
      if (inString.length() < maxLength) {
                inString += inChar; 

      }
      else {
        // empty the string and set it equal to the incoming char:
      //  inString = inChar;
           inString = "";
           inString += inChar;
      }
    }
  }
}

/****************************************************************************
 * Send data to computer once every second.  Data such as temperatures, etc.
 * This allows current settings to be checked by the controlling program
 * and changed if, and only if, necessary.
 * This is quicker that resending data from the controller each second
 * and the Arduino having to read and interpret the results.
 ****************************************************************************/
void doSerialOutput() {
  if (isArtisan) {
    return;
  }

  //send data to logger
  float tt1;
  float tt2;
  
  #ifdef CELSIUS
    tt1 = t1;
    tt2 = t2;
  #else
    tt1 = (t1 * 9 / 5) + 32;
    tt2 = (t2 * 9 / 5) + 32;
  #endif   
   
  Serial.print("t1=");
  Serial.println(tt1,DP);
  
  //Comment out the next two lines if using only one thermocouple
  Serial.print("t2=");
  Serial.println(tt2,DP);

  Serial.print("power%=");
  Serial.println(power * digitalRead(ssrCtrlPin));
  
  Serial.print("motor=");
  Serial.println(currentMotor);
  

  // only need to send these if Arduino controlling by potentiometer
  if (controlBy == arduino)
  {
    Serial.print("pots=");
    Serial.println(potVal);
    Serial.print("lm35=");
    Serial.println(lmTemp);
  }
}

/****************************************************************************
 * Read temperatures from Max6675 chips Sets t1 and t2, -1 if an error
 * occurred.  Max6675 needs 240 ms between readings or will return last
 * value again. I am reading it once per second.
 ****************************************************************************/
void getTemperatures()
{
 
 temp1 = readThermocouple(CS1, calibrate1);
 temp2 = readThermocouple(CS2, calibrate2);	 
  
 if (temp1 > 0.0) 
 {
    tCumulative1 = tCumulative1 + temp1;
     noGoodReadings1 ++;
 }
 if (temp2 > 0.0) 
 {
    tCumulative2 = tCumulative2 + temp2;
    noGoodReadings2 ++;
 }
}

/****************************************************************************
* Purpose to adjust power based on the value from the potentiometer
****************************************************************************/
void updatePot(){
  potVal = analogRead(potPin);
  
  lastReadValue = potVal;
  smoothedValue = (alpha) * smoothedValue + (1-alpha) * lastReadValue;
  potVal = smoothedValue;

  int pow = map(potVal, 15, potMax, 25, 100);
  setPowerLevel(pow);
}

/****************************************************************************
* Purpose to gauge the internal temperature of the box
****************************************************************************/
void updateLM35() {
  analogRead(lmPin); delay(40);
  int reading = analogRead(lmPin);
  lmTemp = reading * 5.0*100.0 / 1023.0;
}

/****************************************************************************
 * Update the values on the LCD
 ****************************************************************************/
void outputLCD() {
  int col2 = 2;
  int col3 = 10;

  if (controlBy == arduino) {
    //lcd.setCursor(0,0); lcd.write(byte(0));
    lcd.setCursor(0,0); lcd.printByte(0);
    lcd.setCursor(0,1); lcd.print(" ");
  }
  else {
    lcd.setCursor(0,0); lcd.print(" ");
    //lcd.setCursor(0,1); lcd.write(byte(1));
    lcd.setCursor(0,1); lcd.printByte(1);
  }

  lcd.setCursor(col2,0);
  lcd.print(t1,DP);
  lcd.write(B11011111);

  lcd.setCursor(col3,0);
  lcd.print(t2,DP);
  lcd.write(B11011111);
  
  lcd.setCursor(col2,1); // row 2
  char powChar[3];
  sprintf(powChar, "%3d", power * digitalRead(ssrCtrlPin));
  lcd.print(powChar);
  lcd.print("%");

  lcd.setCursor(col3,1);
  lcd.print(lmTemp, DP);
  lcd.write(B11011111);
}

/****************************************************************************
 * Called by main loop once every 250 ms
 * Used to read each thermocouple once every 250 ms
 *
 * Once per second averages temperature results, updates potentiometer and outputs data
 * to serial port.
 ****************************************************************************/
void do250msLoop()
{
  getTemperatures();

  if (tcLoopCount > 3)  // once every four loops (1 second) calculate average temp, update Pot and do serial output
  {
    tcLoopCount = 0;

    if (noGoodReadings1 > 0)  t1 = tCumulative1 / noGoodReadings1; else t1 = -1.0;
    if (noGoodReadings2 > 0)  t2 = tCumulative2 / noGoodReadings2; else t2 = -1.0;
    noGoodReadings1 = 0;
    noGoodReadings2 = 0;
    tCumulative1 = 0.0;
    tCumulative2 = 0.0;

    updateLM35();
    doSerialOutput(); // once per second
  }
  tcLoopCount++;

  // only use Pot if in Arduino control.  If Computer control power is set by computer
  if (controlBy == arduino){
    updatePot();
  }

  outputLCD();
}


/****************************************************************************
 * Main loop must not use delay!  PWM heater control relies on loop running
 * at least every 40 ms.  If it takes longer then heater will be on slightly
 * longer than planned. Not a big problem if 1% becomes 1.2%! But keep loop fast.
 * Currently loop takes about 4-5 ms to run so no problem.
 ****************************************************************************/
void loop(){
  controlBy = digitalRead(ctrlPin);

  getSerialInput();// check if any serial data waiting

  // loop to run once every 250 ms to read TC's update Pot etc.
  if (millis() - lastTCTimerLoop >= 250)
  {
    lastTCTimerLoop = millis();
    do250msLoop();  
  }

  doPWM();        // Toggle heater on/off based on power setting

}


/*****************************************************************
 * Read the Max6675 device 1 or 2.  Returns temp as float or  -1.0
 * if an error reading device.
 * Note at least 240 ms should elapse between readings of a device
 * to allow it to settle to new reading.  If not the last reading 
 * will be returned again.
 *****************************************************************/
float readThermocouple(int CS, float calibrate) //device selected by passing in the relavant CS (chip select)
{
  int value = 0;
  int error_tc = 0;
  float temp = 0.0;

  digitalWrite(CS,LOW); // Enable device

  // wait for it to settle
  delayMicroseconds(1);

  /* Cycle the clock for dummy bit 15 */
  digitalWrite(SCKa,HIGH);
  digitalWrite(SCKa,LOW);

  //wait for it to settle
  delayMicroseconds(1);

  /* Read bits 14-3 from MAX6675 for the Temp 
   Loop for each bit reading the value and 
   storing the final value in 'temp' 
   */
  for (int i=11; i>=0; i--){
    digitalWrite(SCKa,HIGH);  // Set Clock to HIGH
    value += digitalRead(SO) << i;  // Read data and add it to our variable
    digitalWrite(SCKa,LOW);  // Set Clock to LOW
  }

  // Read the TC input to check for error
  digitalWrite(SCKa,HIGH); // Set Clock to HIGH
  error_tc = digitalRead(SO); // Read data
  digitalWrite(SCKa,LOW);  // Set Clock to LOW

  digitalWrite(CS,HIGH); // Disable device 1

  value = value + calibrate;					// Add the calibration value

  temp = (value*0.25);						// Multiply the value by 0.25 to get temp in ˚C

  // return -1 if an error occurred, otherwise return temp
  if(error_tc == 0) {
    return temp; 
  } 
  else { 
    return -1.0; 
  }
}
