#include "Arduino.h"
 
#include <SoftwareSerial.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

SoftwareSerial mySoftwareSerial(8, 9); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

const int trig = 6;     // chân trig của HC-SR04
const int echo = 7;     // chân echo của HC-SR04
// DS1302:  RST pin    -> Arduino Digital 2
//          DAT pin   -> Arduino Digital 3
//          CLK pin  -> Arduino Digital 4
#include <virtuabotixRTC.h>
// Creation of the Real Time Clock Object
virtuabotixRTC myRTC(2, 3, 4);
#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   

//  Variables
const int PulseWire = A0;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED13 = 13;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;           // Determine which Signal to "count as a beat" and which to ignore.
                               // Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
  char data;                             // Otherwise leave the default "550" value. 
#define nhiptim A2        
int q=0;                      
PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"
#define doch 10
#define dockc 11
#define xinduong 12
#define nuoc 5
#define rung A5
long times;
void setup() 
{ 

  pinMode(doch, INPUT_PULLUP);
  pinMode(dockc, INPUT_PULLUP);
  pinMode(xinduong, INPUT_PULLUP);
  pinMode(nuoc, INPUT_PULLUP);
  pinMode(nhiptim, INPUT_PULLUP);
  pinMode(rung, OUTPUT);
  Serial.begin(115200);
  delay(100);
  Serial1.begin(9600);
   
  delay(1000);
    pinMode(trig,OUTPUT);   // chân trig sẽ phát tín hiệu
    pinMode(echo,INPUT);    // chân echo sẽ nhận tín hiệu
 // myRTC.setDS1302Time(00, 05, 10, 5, 24, 11, 2022);
  pulseSensor.analogInput(PulseWire);   
  pulseSensor.blinkOnPulse(LED13);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);   

  // Double-check the "pulseSensor" object was created and "began" seeing a signal. 
   if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.  
  }
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
 // myDFPlayer.begin(mySoftwareSerial);
  if (!myDFPlayer.begin(Serial1)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.setTimeOut(1000); //Set serial communictaion time out 500ms

  //----Set volume----
  myDFPlayer.volume(30);  //Set volume value (0~30).
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
 //myDFPlayer.outputDevice(DFPLAYER_DEVICE_U_DISK);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
 myDFPlayer.loop(0);
}

 

void loop() 
{ myRTC.updateTime();
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
   // Serial.println("cm");
   if(distance <= 70){
   int thoigian = map(distance,0,100,50,1000);
   digitalWrite(rung, HIGH);
   delay(thoigian);
   digitalWrite(rung, LOW);
   delay(thoigian);
   }
   else{digitalWrite(rung, LOW);}
   
  if(digitalRead(dockc) == 0){
    myDFPlayer.loop(distance/100+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop(distance/10%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop(distance%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop(12+2);  // sd:/mp3/0001.mp3
  delay(1000);
  }
  if(digitalRead(doch) == 0){
   myDFPlayer.loop(myRTC.hours/10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop(myRTC.hours%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop(10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop(myRTC.minutes/10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop(myRTC.minutes%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop(11+2);  // sd:/mp3/0001.mp3
  delay(1000);
  }
  if(digitalRead(nuoc) == 0){
   myDFPlayer.loop(1);  // sd:/mp3/0001.mp3
  delay(5000); 
  }
  if(digitalRead(xinduong) == 0){
   myDFPlayer.loop(13+2);  // sd:/mp3/0001.mp3
  delay(5000); 
  }
  ////
  int myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
                                               // "myBPM" hold this BPM value now. 
if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened". 
 //Serial.println("♥  A HeartBeat Happened ! "); // If test is "true", print a message "a heartbeat happened".
 ///Serial.print("BPM: ");                        // Print phrase "BPM: " 
 //Serial.println(myBPM);                        // Print the value inside of myBPM. 
  if(digitalRead(nhiptim) ==0){
    int nhiptims = myBPM;
    if(nhiptims-30 < 70){
      ;
    }
    else{
  myDFPlayer.loop((nhiptims-30)/100+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop((nhiptims-30)/10%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.loop((nhiptims-30)%10+2);  // sd:/mp3/0001.mp3
  delay(1000);
 }
}
}
//Serial.println(myRTC.hours);
  delay(20);                    // considered best practice in a simple sketch.
// sprintf(timeChar, "Time: %02d:%02d:%02d",myRTC.hours, myRTC.minutes, myRTC.seconds);
if (Serial.available() > 0) {
  if(q==0){
    // read the incoming byte:
    data = Serial.read();
  // Serial.println(data); 
   
if( data == 'a'){
  myDFPlayer.loop(14+2);
  delay(1300);
}
if( data == 'b'){
  myDFPlayer.loop(15+2);
  delay(1300);
}
if( data == 'c'){
  myDFPlayer.loop(16+2);
  delay(1300);
}
if( data == 'd'){
  myDFPlayer.loop(17+2);
  delay(1300);
}
if( data == 'e'){
  myDFPlayer.loop(18+2);
  delay(1300);
}
if( data == 'f'){
  myDFPlayer.loop(19+2);
  delay(1300);
}
q=1;
   }
   q=0;
}
data = '0';

}
