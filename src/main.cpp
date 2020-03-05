
// Plotclock
// cc - by Johannes Heberlein 2014
// modified for glow clock - Tucker Shannon 2018
// v 1.05
// thingiverse.com/joo   wiki.fablab-nuernberg.de
// thingiverse.com/TuckerPi
// units: mm; microseconds; radians
// origin: bottom left of drawing surface
// time library see http://playground.arduino.cc/Code/time
// RTC  library see http://playground.arduino.cc/Code/time
//               or http://www.pjrc.com/teensy/td_libs_DS1307RTC.html
// Change log:
// 1.01  Release by joo at https://github.com/9a/plotclock
// 1.02  Additional features implemented by Dave (https://github.com/Dave1001/):
//       - added ability to calibrate servofaktor seperately for left and right servos
//       - added code to support DS1307, DS1337 and DS3231 real time clock chips
//       - see http://www.pjrc.com/teensy/td_libs_DS1307RTC.html for how to hook up the real time clock
// 1.03  Fixed the length bug at the servoplotclockogp2 angle calculation, other fixups
// 1.04  Modified for Tuck's glow clock
// 1.05  Modified calibration mode to draw a 4 point square instead
#include <Arduino.h>
#include <Time.h> // see http://playground.arduino.cc/Code/time
#include <TimeLib.h>
#include <Servo.h>
#include "LowPower.h"
//RTC_DS1307 RTC;

// delete or mark the next line as comment if you don't need these
//#define CALIBRATION      // enable calibration mode
#define REALTIMECLOCK    // enable real time clock

#define WISHY 3 // Offset of the Y coordinats of the plate-wisher

// When in calibration mode, adjust the following factors until the servos move exactly 90 degrees
#define SERVOFAKTORLEFT 660
#define SERVOFAKTORRIGHT 550

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
#define SERVOLEFTNULL 1685
#define SERVORIGHTNULL 875

/*
#define SERVOFAKTORLEFT 660
#define SERVOFAKTORRIGHT 550

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
#define SERVOLEFTNULL 1685
#define SERVORIGHTNULL 875
*/

#define SERVOPINLEFT  10
#define SERVOPINRIGHT 9
#define LASER 12
const int wakeUpPin = 3;
const int btnPin = 7;
const int powerTransistor = 6;
const int clockTransistor = 8;


// length of arms
#define L1 35
#define L2 55.1
#define L3 13.2
#define L4 45

//comment the next line out for military 24 hour time setting.
//#define NONMILITARY


// origin points of left and right servo
#define O1X 24-10//22 //point 1
#define O1Y -28-12    //point 1
#define O2X 49-8//47  //point 2
#define O2Y -25-12    //point 2

#ifdef REALTIMECLOCK
#include <ThreeWire.h>  
#include <RtcDS1302.h>
ThreeWire myWire(4,5,2); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);
// for instructions on how to hook up a real time clock,
// see here -> http://www.pjrc.com/teensy/td_libs_DS1307RTC.html
// DS1307RTC works with the DS1307, DS1337 and DS3231 real time clock chips.
// Please run the SetTime example to initialize the time on new RTC chips and begin running.

//#include <Wire.h>
//#include <DS1307RTC.h> // see http://playground.arduino.cc/Code/time
#endif

Servo servo2;  //
Servo servo3;  //

volatile double lastX = 75;
volatile double lastY = 47.5;

int last_min = 0;
String input_string = "";

void setup()
{
  Serial.begin(9600);
  digitalWrite(LASER, LOW);
  pinMode(LASER, OUTPUT);
  pinMode(wakeUpPin, INPUT);
  pinMode(powerTransistor, OUTPUT);
  pinMode(clockTransistor, OUTPUT);
  pinMode(btnPin, INPUT_PULLUP);
  Rtc.Begin();
}

typedef struct ds1302_struct
{
  uint8_t Seconds:4;      // low decimal digit 0-9
  uint8_t Seconds10:3;    // high decimal digit 0-5
  uint8_t CH:1;           // CH = Clock Halt
  uint8_t Minutes:4;
  uint8_t Minutes10:3;
  uint8_t reserved1:1;
  union
  {
    struct
    {
      uint8_t Hour:4;
      uint8_t Hour10:2;
      uint8_t reserved2:1;
      uint8_t hour_12_24:1; // 0 for 24 hour format
    } h24;
    struct
    {
      uint8_t Hour:4;
      uint8_t Hour10:1;
      uint8_t AM_PM:1;      // 0 for AM, 1 for PM
      uint8_t reserved2:1;
      uint8_t hour_12_24:1; // 1 for 12 hour format
    } h12;
  };
  uint8_t Date:4;           // Day of month, 1 = first day
  uint8_t Date10:2;
  uint8_t reserved3:2;
  uint8_t Month:4;          // Month, 1 = January
  uint8_t Month10:1;
  uint8_t reserved4:3;
  uint8_t Day:3;            // Day of week, 1 = first day (any day)
  uint8_t reserved5:5;
  uint8_t Year:4;           // Year, 0 = year 2000
  uint8_t Year10:4;
  uint8_t reserved6:7;
  uint8_t WP:1;             // WP = Write Protect
};

#define bcd2bin(h,l)    (((h)*10) + (l))
#define bin2bcd_h(x)   ((x)/10)
#define bin2bcd_l(x)    ((x)%10)
#define DS1302_SECONDS           0x80
#define DS1302_MINUTES           0x82
#define DS1302_HOURS             0x84
#define DS1302_DATE              0x86
#define DS1302_MONTH             0x88
#define DS1302_DAY               0x8A
#define DS1302_YEAR              0x8C
#define DS1302_ENABLE            0x8E
#define DS1302_TRICKLE           0x90
#define DS1302_CLOCK_BURST       0xBE
#define DS1302_CLOCK_BURST_WRITE 0xBE
#define DS1302_CLOCK_BURST_READ  0xBF
#define DS1302_RAMSTART          0xC0
#define DS1302_RAMEND            0xFC
#define DS1302_RAM_BURST         0xFE
#define DS1302_RAM_BURST_WRITE   0xFE
#define DS1302_RAM_BURST_READ    0xFF
#define DS1302_SCLK_PIN   5    // Arduino pin for the Serial Clock
#define DS1302_IO_PIN     4    // Arduino pin for the Data I/O
#define DS1302_CE_PIN     2    // Arduino pin for the Chip Enable

void _DS1302_start( void)
{
  digitalWrite( DS1302_CE_PIN, LOW); // default, not enabled
  pinMode( DS1302_CE_PIN, OUTPUT);  

  digitalWrite( DS1302_SCLK_PIN, LOW); // default, clock low
  pinMode( DS1302_SCLK_PIN, OUTPUT);

  pinMode( DS1302_IO_PIN, OUTPUT);

  digitalWrite( DS1302_CE_PIN, HIGH); // start the session
  delayMicroseconds( 4);           // tCC = 4us
}


// --------------------------------------------------------
// _DS1302_stop
//
// A helper function to finish the communication.
//
void _DS1302_stop(void)
{
  // Set CE low
  digitalWrite( DS1302_CE_PIN, LOW);

  delayMicroseconds( 4);           // tCWH = 4us
}

void _DS1302_togglewrite( uint8_t data, uint8_t release)
{
  int i;

  for( i = 0; i <= 7; i++)
  {
    // set a bit of the data on the I/O-line
    digitalWrite( DS1302_IO_PIN, bitRead(data, i));  
    delayMicroseconds( 1);     // tDC = 200ns

    // clock up, data is read by DS1302
    digitalWrite( DS1302_SCLK_PIN, HIGH);    
    delayMicroseconds( 1);     // tCH = 1000ns, tCDH = 800ns

    if( release && i == 7)
    {
      // If this write is followed by a read,
      // the I/O-line should be released after
      // the last bit, before the clock line is made low.
      // This is according the datasheet.
      // I have seen other programs that don't release
      // the I/O-line at this moment,
      // and that could cause a shortcut spike
      // on the I/O-line.
      pinMode( DS1302_IO_PIN, INPUT);

      // For Arduino 1.0.3, removing the pull-up is no longer needed.
      // Setting the pin as 'INPUT' will already remove the pull-up.
      // digitalWrite (DS1302_IO, LOW); // remove any pull-up  
    }
    else
    {
      digitalWrite( DS1302_SCLK_PIN, LOW);
      delayMicroseconds( 1);       // tCL=1000ns, tCDD=800ns
    }
  }
}

void DS1302_clock_burst_write( uint8_t *p)
{
  int i;

  _DS1302_start();

  // Instead of the address,
  // the CLOCK_BURST_WRITE command is issued.
  // the I/O-line is not released
  _DS1302_togglewrite( DS1302_CLOCK_BURST_WRITE, false);  

  for( i=0; i<8; i++)
  {
    // the I/O-line is not released
    _DS1302_togglewrite( *p++, false);  
  }
  _DS1302_stop();
}

void setTime(int seconds, int minutes, int hours, int dayofweek, int dayofmonth, int month, int year) {
  ds1302_struct rtc;
  memset ((char *) &rtc, 0, sizeof(rtc));

  rtc.Seconds    = bin2bcd_l( seconds);
  rtc.Seconds10  = bin2bcd_h( seconds);
  rtc.CH         = 0;      // 1 for Clock Halt, 0 to run;
  rtc.Minutes    = bin2bcd_l( minutes);
  rtc.Minutes10  = bin2bcd_h( minutes);
  
  rtc.h24.Hour   = bin2bcd_l( hours);
  rtc.h24.Hour10 = bin2bcd_h( hours);
  rtc.h24.hour_12_24 = 0; // 0 for 24 hour format
  rtc.Date       = bin2bcd_l( dayofmonth);
  rtc.Date10     = bin2bcd_h( dayofmonth);
  rtc.Month      = bin2bcd_l( month);
  rtc.Month10    = bin2bcd_h( month);
  rtc.Day        = dayofweek;
  rtc.Year       = bin2bcd_l( year - 2000);
  rtc.Year10     = bin2bcd_h( year - 2000);
  rtc.WP = 0;  

  // Write all clock data at once (burst mode).
  DS1302_clock_burst_write( (uint8_t *) &rtc);
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}


void set_XY(double Tx, double Ty)
{
  if (digitalRead(LASER)){
    delay(3);
  }
  else{
   delay(0.1);
  }
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); //
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, L4, c);

  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));
}

void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(7 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

  }

  lastX = pX;
  lastY = pY;
}

void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  }
  while ((start + count) > ende);

}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  }
  while ((start + count) <= ende);
}

// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
// The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up
void number(float bx, float by, int num, float scale) {
  scale = scale *1.2;
  by = by * .8;
  switch (num) {

  case 0:
    scale = scale * 0.8;
    by = by + 3;
    drawTo(bx + 12 * scale, by + 6 * scale);
    digitalWrite(LASER, HIGH);
    bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    digitalWrite(LASER, LOW);
    break;
  case 1:
    drawTo(bx + 5 * scale, by + 18 * scale);
    digitalWrite(LASER, HIGH);
    drawTo(bx + 8 * scale, by + 20 * scale);
    drawTo(bx + 8 * scale, by + 0 * scale);
    digitalWrite(LASER, LOW);
    break;
  case 2:
    scale = scale * 0.9;
    by = by + 3;
    drawTo(bx + 2 * scale, by + 15 * scale);
    digitalWrite(LASER, HIGH);
    bogenUZS(bx + 8 * scale, by + 14 * scale, 5 * scale, 3, -0.8, 1);
    drawTo(bx + 3 * scale, by + 0 * scale);
    drawTo(bx + 14 * scale, by + 0 * scale);
    digitalWrite(LASER, LOW);
    break;
  case 3:
    scale = scale * 0.95;
    drawTo(bx + 3 * scale, by + 17 * scale);
    digitalWrite(LASER, HIGH);
    bogenUZS(bx + 6 * scale, by + 15 * scale, 5 * scale, 3, -2, 0.8);
    bogenUZS(bx + 6 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 0.8);
    digitalWrite(LASER, LOW);
    break;
  case 4:

    //scale = scale 0.9;
    //by = by - 2;
    //drawTo(bx + 10 * scale, by + 0 * scale);
    drawTo(bx + 10 * scale, by + 20 * scale);


    digitalWrite(LASER, HIGH);
    drawTo(bx + 10 * scale, by + 0 * scale);
    digitalWrite(LASER, LOW);
    drawTo(bx + 2 * scale, by + 20 * scale);
    digitalWrite(LASER, HIGH);
    drawTo(bx + 2 * scale, by + 10 * scale);
    drawTo(bx + 14 * scale, by + 10 * scale);
    digitalWrite(LASER, LOW);

    break;
  case 5:
    scale = scale * 1.0;
    drawTo(bx + 2 * scale, by + 5 * scale);
    digitalWrite(LASER, HIGH);
    bogenGZS(bx + 5 * scale, by + 8 * scale, 5 * scale, -2.5, 2, 1);
    drawTo(bx + 3 * scale, by + 20 * scale);
    drawTo(bx + 13 * scale, by + 20 * scale);
    digitalWrite(LASER, LOW);
    break;
  case 6:
    //scale = scale * 0.9;
    by = by + 1;
    drawTo(bx + 4 * scale, by + 10 * scale);
    digitalWrite(LASER, HIGH);
    bogenUZS(bx + 7 * scale, by + 6 * scale, 5 * scale, 2, -4.4, 1);
    drawTo(bx + 11 * scale, by + 20 * scale);
    digitalWrite(LASER, LOW);
    break;
  case 7:
    scale = scale * 0.9;
    drawTo(bx + 2 * scale, by + 22 * scale);
    digitalWrite(LASER, HIGH);
    drawTo(bx + 12 * scale, by + 22* scale);
    drawTo(bx + 2 * scale, by + 2);
    digitalWrite(LASER, LOW);

    break;
  case 8:
    scale = scale *0.9;
    drawTo(bx + 5 * scale, by + 10 * scale);

    digitalWrite(LASER, HIGH);
    bogenUZS(bx + 6 * scale, by + 16 * scale, 5 * scale, 4.7, -1.6, 1);
    bogenGZS(bx + 6 * scale, by + 6 * scale, 5 * scale, -4.7, 2, 1);
    digitalWrite(LASER, LOW);
    break;

  case 9:
    drawTo(bx + 9 * scale, by + 11 * scale);

    digitalWrite(LASER, HIGH);
    bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    drawTo(bx + 5 * scale, by + 0);
    digitalWrite(LASER, LOW);
    //lift(1);
    break;



  case 11:
    drawTo(bx + 5 * scale, by + 16 * scale);
    delay(100);
    digitalWrite(LASER, HIGH);
    delay(50);
    digitalWrite(LASER, LOW);
    drawTo(bx + 5 * scale, by + 4 * scale);
    delay(100);
    digitalWrite(LASER, HIGH);
    delay(50);
    digitalWrite(LASER, LOW);
    //lift(1);
    break;

  }
}

String getSplitPart(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void serialLoop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == ';')
    { 
      if (input_string.length() > 15) {
        int seconds = getSplitPart(input_string, ',', 0).toInt();
        int minutes = getSplitPart(input_string, ',', 1).toInt();
        int hours = getSplitPart(input_string, ',', 2).toInt();
        int dayofweek = getSplitPart(input_string, ',', 3).toInt();
        int dayofmonth = getSplitPart(input_string, ',', 4).toInt();
        int month = getSplitPart(input_string, ',', 5).toInt();
        int year = getSplitPart(input_string, ',', 6).toInt();
        setTime(seconds, minutes, hours, dayofweek, dayofmonth, month, year);
        Serial.println("time setting done");
      }
      input_string = "";
      
    }
    else
    {
        input_string += c;
    }
  }
}

void wakeUp()
{
    // Just a handler for the pin interrupt.
}

void loop()
{
  attachInterrupt(1, wakeUp, CHANGE);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
  detachInterrupt(1);

  digitalWrite(powerTransistor, HIGH);
  digitalWrite(clockTransistor, HIGH);
  serialLoop();
  //tmElements_t tm;
  #ifdef CALIBRATION
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);
    // Servohorns will have 90° between movements, parallel to x and y axis
    while (true) {
      drawTo(5, 15);
      delay(2000);
      drawTo(5, 50);
      delay(500);
      drawTo(70, 50);
      delay(500);
      drawTo(70,15);
      delay(500);
    }

  #endif
  int i = 0;

    //if (btn.isSingle()) {
      if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
      if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

      RtcDateTime now = Rtc.GetDateTime();

      //if (RTC.read(tm))
      //{
        setTime(now.Hour(),now.Minute(),now.Second(),now.Day(),now.Month(),now.Year());
        #ifdef NONMILITARY
          if (tm.Hour < 1){
            setTime(12,tm.Minute,tm.Second,tm.Day,tm.Month,tm.Year);
          }
          if (tm.Hour > 12){
            setTime(tm.Hour-12,tm.Minute,tm.Second,tm.Day,tm.Month,tm.Year);
          }
        #endif
      //}
      //setTime(17,01,0,0,0,0);
      hour();
      while ((i+1)*10 <= hour())
      {
        i++;
      }

      if (i != 0){
        number(5, 25, i, 0.9);
      }
      number(21, 25, (hour()-i*10), 0.9);
      number(35, 25, 11, 0.9);

      i=0;
      while ((i+1)*10 <= minute())
      {
        i++;
      }
      number(44, 25, i, 0.9);
      number(57, 25, (minute()-i*10), 0.9);
      //lift(2);
      drawTo(55, 2);
      //lift(1);
      last_min = minute();
      delay(580);

      servo2.detach();
      servo3.detach();
      digitalWrite(powerTransistor, LOW);
      digitalWrite(clockTransistor, LOW);
    //}
  }