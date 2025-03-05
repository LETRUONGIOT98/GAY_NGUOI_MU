#include <SoftwareSerial.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

SoftwareSerial mySoftwareSerial(2, 3); // RX nối với TX DFPlayerMini , TX nối với RX DFPlayerMini
DFRobotDFPlayerMini myDFPlayer;

const int trig = 10;     // chân trig của HC-SR04
const int echo = 11;     // chân echo của HC-SR04     
void setup() 
{ 

  
  Serial.begin(9600);
  delay(100);
  mySoftwareSerial.begin(9600);
   
    pinMode(trig,OUTPUT);   // chân trig sẽ phát tín hiệu
    pinMode(echo,INPUT);    // chân echo sẽ nhận tín hiệu
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
 // myDFPlayer.begin(mySoftwareSerial);
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
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
}

 

void loop() 
{ int distance; 
  for(int i = 0; i <= 50; i++){
  unsigned long duration; // biến đo thời gian
              // biến lưu khoảng cách
    
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
   Serial.println(distance);
  }
  if(distance >0 && distance <= 120){//nếu khoảng cách bé hơn 150cm thfi đọc khoảng cách
    if(distance <= 25){
    myDFPlayer.play(0);  // sd:/mp3/0001.mp3
  delay(2500);
    }
  /*  if(distance >= 10){
  myDFPlayer.play(distance/10%10+1);  // sd:/mp3/0001.mp3
  delay(1000);
    }
  myDFPlayer.play(distance%10+1);  // sd:/mp3/0001.mp3
  delay(1000);
  myDFPlayer.play(12);  // sd:/mp3/0001.mp3
  delay(1000);*/
  }
}
