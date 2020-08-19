/**
   ESPNOW - Basic communication - Slave
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and multiple ESP32 Slaves
   Description: This sketch consists of the code for the Slave module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/
   << This Device Slave >>
   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave(s)
   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor
   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <ADXL345.h>
#include "ITG3200.h"
#include "MovingAverageFilter.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)

#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it

#endif
BluetoothSerial SerialBT;

ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
ITG3200 gyro;
MovingAverageFilter movingAverageFilter(10000);    
int x,y,z;
float xg,yg,zg;
float soh;
float tilt;
float angle;
String dir;
float deltaGz;
float gz_prev=0;
float degx = 0;
float anglex=0,angley=0,anglez=0;
float out1=0;
int out2=0;

uint8_t degree[8]  = {140,146,146,140,128,128,128,128};

#define CHANNEL 1
int H=0;
// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  String Prefix = "Slave:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void setup() {
  Serial.begin(115200);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  SerialBT.begin("ANGULIA001");
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  adxl.powerOn();
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  gyro.init();
  gyro.zeroCalibrate(200,10);//sample 200 times to calibrate and it will take 200*10ms
  pinMode(LED_BUILTIN, OUTPUT);
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  //Serial.print("Last Packet Recv Data: ");
   
  H=*data;
  
}

void loop() {
  //Serial.println (H);
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    float output = 0; 
  //Boring accelerometer stuff   
  adxl.readAccel(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
  xg = x*0.0039;
  yg = y*0.0039;
  zg = z*0.0039;
    if (zg<0.01) {zg=255.60;}
    if (zg>200) {zg=zg-255.60;}
  soh = yg/zg;
  tilt = atan(soh)*57.296;

    if (tilt < 0) {
      tilt= -89.9/tilt*91;
    } 
    //Serial.println("Tilt:");Serial.println(angle);Serial.println(byte(0));Serial.println(dir);
  
int16_t x,y,z;
    gyro.getXYZ(&x,&y,&z);
    float wx,wy,wz;
    char m[6];
    gyro.getAngularVelocity(&wx,&wy,&wz);
    anglex=wx*0.03;
    angley=wy*0.03;
    anglez=wy*0.03;
    //anglex=abs(anglex);
    //degx= (degx+anglex);
    degx = (wx/2000)*(degx + anglex)+(1-(wx/2000))*tilt;
    output = movingAverageFilter.process(degx);
    gz_prev = z;
    out1=abs(H-output);
    out2=H-output;
    out2= abs(out2);
    if (out2>140) {out2=140;}
    sprintf(m,"A#%d#B",out2);
  char *p;
  p = m;
  /*for (p=m; *p != '\0'; p++)
  {
    //Serial.write('\n');
   SerialBT.print(*p);
   Serial.print(*p);
   delay (50);  
  }
  */  
  SerialBT.print(p);
  Serial.println(out2);

  delay (30);
  // Chill
}
