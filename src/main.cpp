#include <Wire.h>

#include "config.h"
#define USE_LVGL

#ifdef USE_LVGL
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#else
#ifdef TFT
#include "Free_Fonts.h" // Include the header file attached to this sketch
#include "TFT_eSPI.h"
#endif
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


#ifdef USE_LVGL
SPIClass touchscreenSpi = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);
uint16_t touchScreenMinimumX = 200, touchScreenMaximumX = 3700, touchScreenMinimumY = 240,touchScreenMaximumY = 3800;

/*Set to your screen resolution*/
#define TFT_HOR_RES   240
#define TFT_VER_RES   320

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))

#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
    LV_UNUSED(level);
    Serial.println(buf);
    Serial.flush();
}
#endif

/* LVGL calls it when a rendered image needs to copied to the display*/
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
    /*Call it to tell LVGL you are ready*/
    lv_disp_flush_ready(disp);
}
/*Read the touchpad*/
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
  if(touchscreen.touched())
  {
    TS_Point p = touchscreen.getPoint();
    //Some very basic auto calibration so it doesn't go out of range
    if(p.x < touchScreenMinimumX) touchScreenMinimumX = p.x;
    if(p.x > touchScreenMaximumX) touchScreenMaximumX = p.x;
    if(p.y < touchScreenMinimumY) touchScreenMinimumY = p.y;
    if(p.y > touchScreenMaximumY) touchScreenMaximumY = p.y;
    //Map this to the pixel position
    data->point.x = map(p.x,touchScreenMinimumX,touchScreenMaximumX,1,TFT_HOR_RES); /* Touchscreen X calibration */
    data->point.y = map(p.y,touchScreenMinimumY,touchScreenMaximumY,1,TFT_VER_RES); /* Touchscreen Y calibration */

    data->state = LV_INDEV_STATE_PRESSED;
    
    Serial.print("Touch x ");
    Serial.print(data->point.x);
    Serial.print(" y ");
    Serial.println(data->point.y);
    
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

lv_indev_t * indev; //Touchscreen input device
uint8_t* draw_buf;  //draw_buf is allocated on heap otherwise the static area is too big on ESP32 at compile
uint32_t lastTick = 0;  //Used to track the tick timer

#else
#ifdef TFT
// Use hardware SPI
TFT_eSPI display = TFT_eSPI();
int ILI9341_COLOR;
#define CUSTOM_DARK 0x4228 // Background color
void spiDisplayTask(void *parameter);
#endif
#endif

#include "ADS1X15.h"

ADS1115 ADS(0x48);

#include "INA226.h"

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
void measureTask(void *parameter);
void lv_tick_task(void * pvParameters);
void ui_init();
static void event_handler(lv_event_t * e);

#define LOGSIZE 100


int packetCounter;
int showConnected = 0;

#define MLEN 8
class Meas_va measurements[MLEN];
bool new_meas;

lv_obj_t *meas_txt[MLEN];

lv_obj_t *label;

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

#ifdef USE_LVGL
ui_init();
#else
  displayInit();
  startUpDisplay();
#endif
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(DEFAULT_SSID);

 // connectingDisplay();

  WiFi.begin(DEFAULT_SSID,  DEFAULT_PASSWORD);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(WiFiEvent);
  


  delay(500);
  #ifdef SPIDISPLAYTASK
  xTaskCreate(spiDisplayTask,   /* Task function. */
              "spiDisplayTask", /* String with name of task. */
              10000,            /* Stack size in words. */
              NULL,             /* Parameter passed as input of the task */
              2,                /* Priority of the task. */
              NULL);            /* Task handle. */
#endif

#ifdef MQTT
mqttInit();
xTaskCreate(mqttTask,   /* Task function. */
              "mqttTask", /* String with name of task. */
              10000,            /* Stack size in words. */
              NULL,             /* Parameter passed as input of the task */
              2,                /* Priority of the task. */
              NULL);            /* Task handle. */
#endif

#ifdef VEDIRECT
xTaskCreate(uartVeDirectTask,   /* Task function. */
                "uartVeDirectTasksTask", /* String with name of task. */
                10000,         /* Stack size in words. */
                NULL,          /* Parameter passed as input of the task */
                5,             /* Priority of the task. */
                NULL); 
#endif

#if 1
xTaskCreate(measureTask,   /* Task function. */
    "measureTask", /* String with name of task. */
    3000,         /* Stack size in words. */
    NULL,          /* Parameter passed as input of the task */
    5,             /* Priority of the task. */
    NULL); 
#endif

#if 0
xTaskCreate(lv_tick_task, 
    "lv_tick_task", 
    1024*10, 
    NULL, 
    8, 
    NULL
);
#endif

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  
}


void ui_init()
{
#ifdef USE_LVGL
String LVGL_Arduino = "LVGL demo ";
LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
Serial.begin(115200);
Serial.println(LVGL_Arduino);
  
//Initialise the touchscreen
touchscreenSpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS); /* Start second SPI bus for touchscreen */
touchscreen.begin(touchscreenSpi); /* Touchscreen init */
touchscreen.setRotation(0); /* Inverted landscape orientation to match screen */

//Initialise LVGL
lv_init();
draw_buf = new uint8_t[DRAW_BUF_SIZE];
lv_display_t * disp;
disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, DRAW_BUF_SIZE);

//Initialize the XPT2046 input device driver
indev = lv_indev_create();
lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);  
lv_indev_set_read_cb(indev, my_touchpad_read);

lv_obj_t * tabview;
    tabview = lv_tabview_create(lv_screen_active());
    lv_tabview_set_tab_bar_size(tabview, 40);


    /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
    lv_obj_t * tab1 = lv_tabview_add_tab(tabview, "Tab 1");
    lv_obj_t * tab2 = lv_tabview_add_tab(tabview, "Tab 2");
    lv_obj_t * tab3 = lv_tabview_add_tab(tabview, "Tab 3");

    lv_obj_t * cont_row = lv_obj_create(tab3 );
    lv_obj_set_size(cont_row, 240, 240);
    lv_obj_align(cont_row, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_COLUMN);



label = lv_label_create( tab1 );
lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
lv_label_set_text( label, "Hello Kate, I'm LVGL!" );
lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );

for(int i=0;i<MLEN;i++) {

  lv_obj_t *mtxt = lv_label_create( cont_row);
  lv_obj_set_style_text_font(mtxt, &lv_font_montserrat_20, 0);
  lv_label_set_text( mtxt, "mtxt " );
  lv_obj_align( mtxt, LV_ALIGN_CENTER, 0, 0 );
  meas_txt[i]=mtxt;
}



lv_obj_t * btn1 = lv_button_create(tab2);
lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

lv_obj_t * blabel = lv_label_create(btn1);
lv_label_set_text(blabel, "Button");
lv_obj_center(blabel);

#endif

}


static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {
        LV_LOG_USER("Clicked");
    }
    else if(code == LV_EVENT_VALUE_CHANGED) {
        LV_LOG_USER("Toggled");
    }
}


void measureTask(void *parameter)
{
  //httpServer.begin();
  Serial.println("maesureTask herr !!");
  INA226 INA0(0x40);
  INA226 INA1(0x41);
  i2cscan();
  ADS.begin();
  ADS.setGain(1); // +/-4.096V range
  float vi,ai,si;
  float v0,v1,v2,v3;
  float i0,i1,i2,i3;

  bool ina0_b=INA0.begin();
  bool ina1_b=INA1.begin();

  if (!ina0_b)
  {
    Serial.println("INA0 could not connect");
  } else {
    INA0.setMaxCurrentShunt(1, 0.01);
  }
  if (!ina1_b )
  {
    Serial.println("INA1 could not connect");
  }
  else {
    INA1.setMaxCurrentShunt(1, 0.01);
  }
  float factor = ADS.toVoltage(1);  //  voltage factor
  //int16_t value0 = ADS.readADC(0);
  //int16_t value1 = ADS.readADC(1);
  
  while(1) {
    if(ina0_b) {
      vi =INA0.getBusVoltage();
      ai =INA0.getCurrent();
      si =INA0.getShuntVoltage();
      measurements[0].V=vi;
      measurements[0].A=vi;
      Serial.printf("v=%2.1f a=%2.1f sv=%2.1f \n",vi,ai,si);
    };
    if(ina0_b) {
      vi =INA1.getBusVoltage();
      ai =INA1.getCurrent();
      si =INA1.getShuntVoltage();
      measurements[1].V=vi;
      measurements[1].A=vi;
      Serial.printf("v=%2.1f a=%2.1f sv=%2.1f \n",vi,ai,si);
    };
    
    
    float v0 =factor*ADS.readADC(0);
    float v1 =factor*ADS.readADC(1);
    float v2 =factor*ADS.readADC(2);
    float v3 =factor*ADS.readADC(3);
     
    i0 =(v0-2.489)*50.97;
    i1 =(v1-2.489)*50.97;
    i2 =(v2-2.489)*50.97;
    i3 =(v3-2.489)*50.97;

    measurements[2].A=i0;
    measurements[3].A=i1;
    measurements[4].A=i2;
    measurements[5].A=i3;
    measurements[2].V=v0;
    measurements[3].V=v1;
    measurements[4].V=v2;
    measurements[5].V=v3;



   
    new_meas=true;
    delay(500);
  }
  
}


void lv_tick_task(void * pvParameters)
{
    while (1)
    {
        lv_task_handler();
        vTaskDelay((20) / portTICK_PERIOD_MS);
    }
}


void loop()
{
  char s[100];
  //httpServer.WifiLoop();
  if (!WiFi.isConnected())
  {
    Serial.println("trying to reconnect Wifi");
    WiFi.reconnect();
    delay(1500);
  } else {
    lv_tick_inc(millis() - lastTick); //Update the tick timer. Tick is new for LVGL 9
    lastTick = millis();
    if(new_meas) {
      for(int i=0;i<MLEN;i++) {
        sprintf(s,"v=%2.3f a=%2.3f",measurements[i].V,measurements[i].A);
        lv_label_set_text( meas_txt[i], s );
      }
      new_meas=false;
    }
    lv_timer_handler();               //Update the UI
    delay(5);
    //mqttclient.loop();
  }
 
  
  
  //Serial.printf("Analog0 v0=% 3.3f i0=% 5.2f v1=% 3.3f vi=% 3.3f % 3.3f % 3.3f\n",v0,i0,v1,vi,ai,si*100);


}

#ifdef VEDIRECT
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
#endif

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
