#include <Wire.h>

#include "config.h"

#ifdef OLED
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
//#include "OZ890.h"
#include "OLEDDisplayUi.h"
#endif

#ifdef TFT
#include "Free_Fonts.h" // Include the header file attached to this sketch
#include "TFT_eSPI.h"

#endif

#include <WiFi.h>
#include "FS.h"
#include <SD.h>
#include "SPIFFS.h"
//#include "configfile.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
//#include "mqtt.h"

#include "measurements.h"
#include "VeDirectDCDC.h"

#ifdef TFT
// Use hardware SPI
TFT_eSPI display = TFT_eSPI();
int ILI9341_COLOR;
#define CUSTOM_DARK 0x4228 // Background color
void spiDisplayTask(void *parameter);
#endif

#include "ADS1X15.h"

ADS1115 ADS(0x48);

#include "INA226.h"

INA226 INA0(0x40);


const char *ssid = DEFAULT_SSID;
const char *password = DEFAULT_PASSWORD;

void i2cTask(void *parameter);
int i2cscan();

WiFiClient espClient;
PubSubClient mqttclient(espClient);

void displayInit();
void startUpDisplay();
void connectingDisplay();
void uartVeDirectTask(void *parameter);

#define LOGSIZE 100


int packetCounter;
int showConnected = 0;


float v0,v1,vi,ai,si,i0;

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    showConnected = 1;
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    break;
  }
}

void setup()
{
  int a=I2C_SDA;
  int b=I2C_SCL;

  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL, 80000);
  Serial.printf("i2c initialized I2C_SDA=%d, I2C_SCL=%d\n",I2C_SDA, I2C_SCL);

  //pinMode(BUTTON, INPUT);
  //digitalWrite(BUTTON, HIGH);

  //   Wire.begin(I2C_SDA,I2C_SCL,300000);

  
  displayInit();

  // Wire.setClock(300000);
  Serial.println();

  startUpDisplay();

  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(DEFAULT_SSID);

  connectingDisplay();

  WiFi.begin(DEFAULT_SSID,  DEFAULT_PASSWORD);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(WiFiEvent);
  


  delay(500);
  xTaskCreate(spiDisplayTask,   /* Task function. */
              "spiDisplayTask", /* String with name of task. */
              10000,            /* Stack size in words. */
              NULL,             /* Parameter passed as input of the task */
              2,                /* Priority of the task. */
              NULL);            /* Task handle. */


#ifdef MQTT
mqttInit();
xTaskCreate(mqttTask,   /* Task function. */
              "mqttTask", /* String with name of task. */
              10000,            /* Stack size in words. */
              NULL,             /* Parameter passed as input of the task */
              2,                /* Priority of the task. */
              NULL);            /* Task handle. */
#endif

xTaskCreate(uartVeDirectTask,   /* Task function. */
                "uartVeDirectTasksTask", /* String with name of task. */
                10000,         /* Stack size in words. */
                NULL,          /* Parameter passed as input of the task */
                5,             /* Priority of the task. */
                NULL); 

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  //httpServer.begin();
  i2cscan();
  ADS.begin();
  ADS.setGain(1); // +/-4.096V range

  if (!INA0.begin() )
  {
    Serial.println("INA0 could not connect. Fix and Reboot");
  }
}

void loop()
{
  //httpServer.WifiLoop();
  if (!WiFi.isConnected())
  {
    Serial.println("trying to reconnect Wifi");
    WiFi.reconnect();
    delay(1500);
  } else {
    //mqttclient.loop();
  }
 
  float factor = ADS.toVoltage(1);  //  voltage factor
  //int16_t value0 = ADS.readADC(0);
  //int16_t value1 = ADS.readADC(1);
  INA0.setMaxCurrentShunt(1, 0.01);
  vi =INA0.getBusVoltage();
  ai =INA0.getCurrent();
  si =INA0.getShuntVoltage();

  float vx =factor*ADS.readADC(0);
  //v1 =factor*ADS.readADC(1);
  if (vx>0.1) v0=vx;
  v1=0.0;
  i0 =(v0-1.726)*47.4;
  delay(50000);
  
  
  //Serial.printf("Analog0 v0=% 3.3f i0=% 5.2f v1=% 3.3f vi=% 3.3f % 3.3f % 3.3f\n",v0,i0,v1,vi,ai,si*100);


}


bool vedirectInit()
{
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
}
void vedirectRead(ViDirectDcDc &vd);

void uartVeDirectTask(void *parameter)
{
  String s, s2;
  double vTmp;

  Serial.println("Created uartVeDirectTask:");
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Created uartVeDirectTask:2");
  delay(1500);
  //vedirectInit();

  ViDirectDcDc vd;

  while (1)
  {
    
    vedirectRead(vd);
    delay(1500);
  }
  Serial.println("exit uartVeDirectTask:");
}


void hex_dump(const void *data, int size) {
  const unsigned char *bytes = (const unsigned char *)data;
  size_t i, j;

  for (i = 0; i < size; i += 16) {
      // Print address
      Serial.printf("%04x  ", i);

      // Print hex bytes
      for (j = 0; j < 16; ++j) {
          if (i + j < size)
          Serial.printf("%02x ", bytes[i + j]);
          else
          Serial.printf("   "); // Padding for alignment
      }

      printf(" ");

      // Print ASCII characters
      for (j = 0; j < 16 && (i + j) < size; ++j) {
          unsigned char c = bytes[i + j];
          Serial.printf("%c", isprint(c) ? c : '.');
      }

      Serial.printf("\n");
  }
}




ViDirectDcDc::ViDirectDcDc()
{
  in_v=0;
  in_a=0;
  in_w=0;
  out_v=0;
  out_a=0;
  out_w=0;
  cs=0;
  zor=0;
  er=0;
};

void  ViDirectDcDc::print()
{
  Serial.printf("in=v=%f in_a=%f out_v=%f out_a=%f\n",in_v,in_a,out_v,out_a);
}

void  ViDirectDcDc::parse(unsigned char buffer[],int p)
{
  int i=0;
  int j=0;
  int v=0;
  int k=0;
  while(i<p) {
    if((i<p) && (buffer[i]==0x0a)) { // FInd a Line
      buffer[i]=0;
      j=i+1;
      k=j;
      while((buffer[j]!=0x0a) && (j<p)) {
        if(buffer[j]==0x0d) buffer[j]=0;
        if(buffer[j]==0x09) {
          v=j+1;
          buffer[j]=0;
        } 
        j++;
      };
      buffer[j]=0;
      Serial.printf("key=%s val=%s\n",&buffer[k],&buffer[v]);
      if(strncmp((const char*)&buffer[k],"DC_IN_V",10)==0) in_v=atoi((const char*)&buffer[v])/100.0;
      if(strncmp((const char*)&buffer[k],"V",10)==0) out_v=atoi((const char*)&buffer[v])/1000.0;
      if(strncmp((const char*)&buffer[k],"I",10)==0) out_a=atoi((const char*)&buffer[v])/1000.0;
      if(strncmp((const char*)&buffer[k],"DC_IN_I",10)==0) in_a=atoi((const char*)&buffer[v])/10.0;
      if(strncmp((const char*)&buffer[k],"P",10)==0) out_w=atoi((const char*)&buffer[v]);
      if(strncmp((const char*)&buffer[k],"DC_IN_P",10)==0) in_w=atoi((const char*)&buffer[v]);
    } else i++;
  }
}



void vedirectRead(ViDirectDcDc &vd)
{
  ;
  unsigned char buffer[256];
  int p;
  p=0;
  while (Serial2.available() && p<255)
  {
    char c = Serial2.read();
    //lastReadMs=millis();
    buffer[p++] = c;
  } 
  if(p>0) {
    buffer[p]=0;
    //Serial.printf("Read %p %s",p,&buffer);
    //hex_dump(&buffer, p);
    vd.parse((unsigned char*)&buffer,p);
    vd.print();
  }
}


int i2cscan()
{
  byte error, address;
  int nDevices;

  //Serial.println("Scanning.. SDA:"+String(Wire.sda)+" SCL:"+String(Wire.scl)+" "+String(Wire.num));

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  return nDevices;
}
