#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
char auth[] = "PcGd35_r7PtmESE_FtDBlnmo4b8NNXhv";
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Thu Thao";  //Tên wifi
char pass[] = "12052002"; //Mật khẩu wifi
WidgetMap myMap(V7);
BLYNK_WRITE(V1) {
  GpsParam gps(param);

  // Print 6 decimal places for Lat, Lon
  Serial.print("Lat: ");
  Serial.println(gps.getLat(), 7);



  Serial.print("Lon: ");
  Serial.println(gps.getLon(), 7);

  // Print 2 decimal places for Alt, Speed
  Serial.print("Altitute: ");
  Serial.println(gps.getAltitude(), 2);

  Serial.print("Speed: ");
  Serial.println(gps.getSpeed(), 2);
myMap.location(1, gps.getLat(), gps.getLon(), "VI TRI NGUOI KHIEM THI");
  Serial.println();
}
void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass,"Blynk-server.com",8080);
}

void loop() {
  Blynk.run();
} 
