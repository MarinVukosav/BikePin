
#include <GSM.h>
#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include <Time.h>
#include <DS3232RTC.h>
#include <LowPower.h>
#include <TimeLib.h>

//GSM
#define PINNUMBER ""
GSM gsmAccess;
GSM_SMS sms;
int readSerial(char result[]);
char senderNumber[20];
static const int RXPin = 9, TXPin = 8;
static const int GPSBaud = 9600;

//GPS
TinyGPSPlus gps;
static void printStr(const char *str, int len);
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
static void printInt(unsigned long val, bool valid, int len);
static void printFloat(float val, bool valid, int len, int prec);
int readSerial(char result[]);
static void smartDelay(unsigned long ms);

//RTC
#define RTC_INTERRUPT_PIN 2
#define D_SLEEP_PERIOD 60
#define D_BACKOFF_TIMEOUT 1500
#define D_RETRY_LIMIT 4
#define D_CONTENTION_WINDOW_MAX 100
#define D_ACK_TIMEOUT 255
#define ALARM_DELTA_MIN 2
#define RTC_ATTACH_INTERRUPT_DELAY 200
typedef void (*function_ptr)(void);
bool RTCinit(uint8_t interruptPin, function_ptr alarmISR);
time_t RTCsetNewTime(time_t dataTime);
void RTCsleepNow();
void RTCprint(time_t t);
time_t RTCsetNextAlarm(uint16_t sleepPeriod);
volatile boolean alarmISRWasCalled = false;
void alarmISR(void)
{
    alarmISRWasCalled = true;
}

// The serial connection to the GPS device
AltSoftSerial altSerial(RXPin, TXPin);
char remoteNum[20]="0919427728";
int k=0;
void setup() {
  Serial.begin(9600);
  RTCinit(RTC_INTERRUPT_PIN, alarmISR);
  while (!Serial) {
    ;
  }

  Serial.println("SMS Messages Sender");
  // connection state
  boolean notConnected = true;

  // Start GSM shield
  while (notConnected) {
    if (gsmAccess.begin() == GSM_READY) {
      notConnected = false;
    } else {
      Serial.println("Not connected");
      delay(1000);
    }
  }
  Serial.println("GSM initialized");
}

void loop() {
  char c[80];

  char x[15]="GPS";

  if (sms.available()) {
    Serial.println("Message received from:");

    sms.remoteNumber(senderNumber, 20);
    Serial.println(senderNumber);

    int i=0;
    while ((c[i] = sms.read()) ) {
      Serial.print(c[i]);
      i++;
    }

  Serial.println("\nEND OF MESSAGE");

    // Delete message from modem memory
  sms.flush();
  Serial.println("MESSAGE DELETED");


  Serial.print("Enter a mobile number: ");
  // char remoteNum[20]="0919427728";  // telephone number to send sms
  Serial.println(remoteNum);


  // send the message
if( strcmp(x,c) == 0){
  altSerial.begin(GPSBaud);
do{
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);

    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    delay(200);
  }while(gps.location.lat()<=0);
  for (int i = 0; i < 50; i ++)
  {
      Serial.print("Redni broj: ");
      Serial.println(i);
    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);

      printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
      printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
      smartDelay(1500);
        Serial.println(" ");


  }
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

 smartDelay(1500);
 if (gps.location.lat()>0){
     if(RTCsetNewTime(gps.time.value() ) ){
                 Serial.print(F("Setting up new RTC time -> SUCCESS "));
                 Serial.print(F("New time is: "));
               RTCprint(gps.time.value());
      }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  Serial.println(F("No GPS data received: check wiring"));

  Serial.print("Now, entering SMS content: ");
  char str1[16];
  char str2[16];
  dtostrf(gps.location.lat(), 10, 6, str1);
  dtostrf(gps.location.lng(), 10, 6, str2);

      char str3[4]= ",";
      char str [160] = "https://www.google.com/maps/place/";
      strcat(str, str1);
      strcat(str, str3);
      strcat(str, str2);

      Serial.println("SENDING");
      Serial.println();
      Serial.println("Message:");
      Serial.println(str);
      // Serial.println(gps.location.lng());
      sms.beginSMS("0919427728");
      sms.print(str);
      sms.endSMS();
      delay(1000);
      Serial.println("\nCOMPLETE!\n");

}
else{
    Serial.print("nije unesena kljucna rijec");
    char str4[30] = "Kljucna rijec za lociranje je: GPS.";
    sms.beginSMS("0919427728");
    sms.print(str4);
    sms.endSMS();
    delay(1000);
}
time_t nextAlarm = RTCsetNextAlarm(D_SLEEP_PERIOD);
Serial.print(F("Next alarm: "));
RTCprint(nextAlarm);
delay(50);
RTCsleepNow();
}

}

int readSerial(char result[]) {
  int i = 0;
  while (1) {
    while (Serial.available() > 0) {
      char inChar = Serial.read();
      if (inChar == '\n') {
        result[i] = '\0';
        Serial.flush();
        return 0;
      }
      if (inChar != '\r') {
        result[i] = inChar;
        i++;
      }
    }
  }
}


  //------------FUNKCIJE---------------
 static void smartDelay(unsigned long ms)
  {
    unsigned long start = millis();
    do
    {
      while (altSerial.available())
        gps.encode(altSerial.read());
    } while (millis() - start < ms);
  }

  static void printFloat(float val, bool valid, int len, int prec)
  {
    if (!valid)
    {
      //while (len-- > 1)
      //  Serial.print('*');
    //  Serial.print(' ');
    }
    else
    {
    //  Serial.print(val, prec);
      int vi = abs((int)val);
      int flen = prec + (val < 0.0 ? 2 : 1); // . and -
      flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    //  for (int i=flen; i<len; ++i)
    //    Serial.print(' ');
    }
    smartDelay(0);
  }

  static void printInt(unsigned long val, bool valid, int len)
  {
    char sz[32] = "*****************";
    if (valid)
      sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i=strlen(sz); i<len; ++i)
      sz[i] = ' ';
    if (len > 0)
      sz[len-1] = ' ';
    Serial.print(sz);
    smartDelay(0);
  }

  static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
  {
    if (!d.isValid())
    {
      Serial.print(F("********** "));
    }
    else
    {
      char sz[32];
      sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
      Serial.print(sz);
    }

    if (!t.isValid())
    {
      Serial.print(F("******** "));
    }
    else
    {
      char sz[32];
      sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
      Serial.print(sz);
    }

    printInt(d.age(), d.isValid(), 5);
    smartDelay(0);
  }

  static void printStr(const char *str, int len)
  {
    int slen = strlen(str);
    for (int i=0; i<len; ++i)
      Serial.print(i<slen ? str[i] : ' ');
     smartDelay(0);
  }

  bool RTCinit(uint8_t interruptPin, function_ptr alarmISR)
  {
      setSyncProvider(RTC.get);

      if(timeStatus() != timeSet) {
          return false;
      }

      //Disable the default square wave of the SQW pin.
      RTC.squareWave(SQWAVE_NONE);

      //Attach an interrupt on the falling of the SQW pin.
      pinMode(interruptPin, INPUT_PULLUP);
      attachInterrupt(INT0, alarmISR, FALLING);
      delay(RTC_ATTACH_INTERRUPT_DELAY);

      return true;
  }

  time_t RTCsetNewTime(time_t dataTime)
  {
      RTC.set(dataTime);
      setTime(dataTime);
      if(timeStatus() != timeSet) {
          return false;
      }
      else
          return true;
  }

  time_t RTCsetNextAlarm(uint16_t sleepPeriod)
  {
    time_t alarmTime;
    time_t currentTime = RTC.get();

    unsigned long countIntervals = currentTime / sleepPeriod;
    alarmTime = countIntervals*sleepPeriod;

    long delta = alarmTime - currentTime;

    if (delta > 0) {
          if (delta < ALARM_DELTA_MIN) {
              alarmTime += sleepPeriod;
          }
    } else {
         uint8_t shiftAlarm = ((sleepPeriod + delta) < ALARM_DELTA_MIN) ? 1 : 0;
         alarmTime = alarmTime + (1 + shiftAlarm)*sleepPeriod;
    }

    // RTC alarm reset
    while (alarmTime <= RTC.get()) {
          alarmTime += sleepPeriod;
    }

    if ((alarmTime - RTC.get()) > 2*sleepPeriod) {
          alarmTime = RTC.get() + sleepPeriod;
    }

    RTC.setAlarm(ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 1);
    RTC.alarm(ALARM_1);
    RTC.alarmInterrupt(ALARM_1, true);

    delay(RTC_ATTACH_INTERRUPT_DELAY);

    return alarmTime;
  }


  void RTCsleepNow()
  {
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
      delay(50);
  }


  void RTCprint(time_t t)
  {
      Serial.print(((hour(t)<10)  ? "0" : "")); Serial.print(hour(t));  Serial.print(':');
      Serial.print(((minute(t)<10) ? "0" : "")); Serial.print(minute(t)); Serial.print(':');
      Serial.print(((second(t)<10) ? "0" : "")); Serial.println(second(t));
      delay(200);
  }
