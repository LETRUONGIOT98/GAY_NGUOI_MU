#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <DFPlayerMini_Fast.h>

DFPlayerMini_Fast myMP3;

const int trig = 34;     // chân trig của HC-SR04
const int echo = 35;     // chân echo của HC-SR04
// DS1302:  RST pin    -> Arduino Digital 10
//          DAT pin   -> Arduino Digital 9
//          CLK pin  -> Arduino Digital 8
#include "DS1302.h" // include the DS1302 RTC library
 
const int ResetPin = 10;  // reset Pin
const int DataPin = 9;  // data Pin
const int ClockPin = 8;  // clock Pin
DS1302 rtc(ResetPin, DataPin, ClockPin);
#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   

//  Variables
const int PulseWire = A3;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED13 = 13;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 520;           // Determine which Signal to "count as a beat" and which to ignore.
                               // Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
  char data;                             // Otherwise leave the default "550" value. 
#define nhiptim 22        
int q=0;                      
PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"
#define doch 23
#define dockc 24
#define xinduong 25
#define nuoc A0
#define rung 12
#define goi 26
#define nghe 27
#define doctien 28
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
String dayAsString(const Time::Day day) { // function that converts the day ouptput into a string
  switch (day) {
    case Time::kSunday: return "Sunday";
    case Time::kMonday: return "Monday";
    case Time::kTuesday: return "Tuesday";
    case Time::kWednesday: return "Wednesday";
    case Time::kThursday: return "Thursday";
    case Time::kFriday: return "Friday";
    case Time::kSaturday: return "Saturday";
  }
  return "(unknown day)";
}
void setup()
{ pinMode(doch, INPUT_PULLUP);
  pinMode(dockc, INPUT_PULLUP);
  pinMode(xinduong, INPUT_PULLUP);
  pinMode(nuoc, INPUT_PULLUP);
  pinMode(nhiptim, INPUT_PULLUP);
  pinMode(goi, INPUT_PULLUP);
  pinMode(nghe, INPUT_PULLUP);
  pinMode(doctien, INPUT_PULLUP);
  pinMode(rung, OUTPUT);
  pinMode(trig,OUTPUT);   // chân trig sẽ phát tín hiệu
    pinMode(echo,INPUT);    // chân echo sẽ nhận tín hiệu
    rtc.writeProtect(false); // turn off write protection
  rtc.halt(false); // clear the clock halt flag
  Time t(2023, 1, 8,12, 31, 00, Time::kSunday); // create a new time object with set date
  rtc.time(t);
  pulseSensor.analogInput(PulseWire);   
  pulseSensor.blinkOnPulse(LED13);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);   
  Serial.begin(115200);
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.  
  }
  Serial1.begin(9600);
  Serial2.begin(115200);
  myMP3.begin(Serial1, true);
  Serial.println("Setting volume to max");
  myMP3.volume(30);
}
long time1,time2,times,time3,time4 ;
bool tt = true;
void loop()
{
Time t = rtc.time();

  unsigned long duration; // biến đo thời gian
    int distance;           // biến lưu khoảng cách
    
    /* Phát xung từ chân trig */
    digitalWrite(trig,0);   // tắt chân trig
    delayMicroseconds(2);
    digitalWrite(trig,1);   // phát xung từ chân trig
    delayMicroseconds(5);   // xung có độ dài 5 microSeconds
    digitalWrite(trig,0);   // tắt chân trig
    
    /* Tính toán thời gian */
    // Đo độ rộng xung HIGH ở chân echo. 
    duration = pulseIn(echo,HIGH);  
    // Tính khoảng cách đến vật.
    distance = int(duration/2/29.412);
    
    /* In kết quả ra Serial Monitor */
    //Serial.print(distance);
   //Serial.println("cm");
   if(distance <= 70 && distance > 0){
   int thoigian = map(distance,0,70,255,0);
   analogWrite(rung, thoigian);
   }
   else{digitalWrite(rung, LOW);}
   
  if(digitalRead(dockc) == 0){
    myMP3.loop(distance/100+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop(distance/10%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop(distance%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop(12+2);  // sd:/mp3/0001.mp3
  delay(1000);
  }
  Serial.println("GIO: " + String(t.hr));
  Serial.println("PHUT: " + String(t.min));
  if(digitalRead(doch) == 0){
   myMP3.loop(t.hr/10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop(t.hr%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop(10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop(t.min/10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop(t.min%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop(11+2);  // sd:/mp3/0001.mp3
  delay(1000);
  }
  if(digitalRead(nuoc) == 0){
   myMP3.loop(1);  // sd:/mp3/0001.mp3
  delay(5000); 
  }
  if(digitalRead(xinduong) == 0){
   myMP3.loop(13+2);  // sd:/mp3/0001.mp3
  delay(5000); 
  }
  ////
  if(millis() - time2 >= 20){
  int myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
    time2 = millis();
   
                                               // "myBPM" hold this BPM value now. 
//if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened". 
 //Serial.println("♥  A HeartBeat Happened ! "); // If test is "true", print a message "a heartbeat happened".
 Serial.print("BPM: ");                        // Print phrase "BPM: " 
Serial.println(myBPM);                        // Print the value inside of myBPM.  
Serial.println(analogRead(A3));
  int nhiptims = myBPM;
  if(nhiptims > 140) nhiptims = 0;
  if(nhiptims < 90) nhiptims = 0;
  
  if(nhiptims > 90 && nhiptims < 170){
    if(tt){
      tt = false;
  myMP3.loop((nhiptims-30)/100+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop((nhiptims-30)/10%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myMP3.loop((nhiptims-30)%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  }
}
else tt =true;
}

if(digitalRead(goi) == 0){
  if(millis() - times >= 3000 && millis() - times < 6000 ){
    Serial.println("GOI");
 Serial2.println("ATD+84342377970;");//
  delay(3000);
  }
}
if(digitalRead(goi) == 0){
  if(millis() - times < 2000){
    Serial.println("KET THUC");
   Serial2.println("ATH;");
    delay(2000);
  }
}
if(digitalRead(goi) == 1){
  times = millis();
  }
if(digitalRead(nghe) == 0){
  Serial.println("KET THUC");
  Serial2.println("ATA");
  delay(1000);
}

uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
  //Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  //Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
 ////////////////////// Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  //////////////Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  ///////////////Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  //Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  ///////////Serial.println(" ");
 
 /* if(r >= 9000 && r <=9700 && g >= 9700 && g <= 11500 && b >= 8000 && b <= 9100){
    Serial.println("TO 10K");
    if(millis() - time4 > 1000){
    myMP3.loop(14+2);
  delay(1300);
  }
  }*/
   if(r >= 5500 && r <= 7500 && g >= 9000 && g <= 11000 && b >= 10000 && b <= 12000){
    Serial.println("TO 20K");
    if(millis() - time4 > 1000){
    myMP3.loop(15+2);
  delay(1300);
  }}
  else if(r >= 8500 && r <= 9500 && g >= 9500 && g <= 10500 && b >= 9200 && b <= 10000){
    Serial.println("TO 50K");
    if(millis() - time4 > 1000){
    myMP3.loop(16+2);
  delay(1300);
  }}
  else if(r >= 8500 && r <= 9500 && g >= 12500 && g <= 14500 && b >= 9500 && b <= 10500){
    Serial.println("TO 100K");
    if(millis() - time4 > 1000){
    myMP3.loop(17+2);
  delay(1300);
  }}
 /* else if(r >= 9800 && r <= 11000 && g >= 11000 && g <= 12000 && b >= 9000 && b <= 9800){
    Serial.println("TO 200K");
    if(millis() - time4 > 1000){
    myMP3.loop(18+2);
  delay(1300);
  }}*/
  else if(r >= 7000 && r <= 8000 && g >= 12000 && g <= 13000 && b >= 11000 && b <= 12000){
    Serial.println("TO 500K");
    if(millis() - time4 > 1000){
    myMP3.loop(19+2);
  delay(1300);
  }}
  else time4 = millis(); 
}


/*
   mp3_play ();   //start play
   mp3_play (5);  //play "mp3/0005.mp3"
   mp3_next ();   //play next
   mp3_prev ();   //play previous
   mp3_set_volume (uint16_t volume);  //0~30
   mp3_set_EQ (); //0~5
   mp3_pause ();
   mp3_stop ();
   void mp3_get_state ();   //send get state command
   void mp3_get_volume ();
   void mp3_get_u_sum ();
   void mp3_get_tf_sum ();
   void mp3_get_flash_sum ();
   void mp3_get_tf_current ();
   void mp3_get_u_current ();
   void mp3_get_flash_current ();
   void mp3_single_loop (boolean state);  //set single loop
   void mp3_DAC (boolean state);
   void mp3_random_play ();
*/
