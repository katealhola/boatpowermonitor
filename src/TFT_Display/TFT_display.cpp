#include "config.h"
#include <WiFi.h>

#define TFT
#ifdef TFT
#include "Free_Fonts.h" // Include the header file attached to this sketch
#include <Fonts/GFXFF/gfxfont.h>
#include "Mono16.h"
#include "Dialog28.h"
#include "Dialog32.h"

#include "TFT_display.h"

//#include "SPI.h"
#include "TFT_eSPI.h"
#include "TFTShape.h"
extern TFT_eSPI  display ;
#define CUSTOM_DARK 0x4228 // Background color
#define TFT_GREY 0x5AEB
#define TFT_ORANGE      0xFD20      /* 255, 165,   0 */

#define M_SIZE 0.6


extern const char* ssid;

float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = M_SIZE*120, osy = M_SIZE*120; // Saved x & y coords
uint32_t updateTime = 0;       // time for next update


void displayInit()
{
   display.init();
#ifdef SMALLDISPLAY
  display.setRotation(1);
#else
 display.setRotation(0);
#endif
  display.fillScreen(TFT_BLACK);
  display.setTextColor(TFT_WHITE);
  display.setTextWrap(true);
  display.setCursor(0, 0);
  display.setFreeFont(FSB18); 
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  display.setTextColor(TFT_GREEN, TFT_BLACK);
  display.fillScreen(TFT_BLACK);
  display.setTextDatum(MC_DATUM);
}

void startUpDisplay()
{
   display.fillScreen(CUSTOM_DARK);
  display.setTextColor(TFT_WHITE);
  display.setCursor(0, 0);
  display.println();
  display.setFreeFont(FSB18); 
  display.print("KatePowerMonitor");
}

void connectingDisplay()
{
 /*display.fillScreen(CUSTOM_DARK);
  display.setTextColor(TFT_WHITE);
  display.setCursor(20, 0);*/
  display.println();  
  display.setFreeFont(FSB12); 
  display.print("connecting to ");
  display.print(ssid); 
}


void displayString(int x, int y, String s)
{
  display.setCursor(x + 20, y);
  display.println(s);
}

void networkDisplay(TFT_eSprite &m )
{
  if (!WiFi.isConnected()) {
    m.println("trying to reconnect Wifi");
    m.println(String("ssid ") + ssid);
  } else
  {
    m.println("connected");
    m.println(String("ssid ") + ssid);
    String ips = WiFi.localIP().toString();
    m.println(ips);
  }
}

int valueToPixels(float val, float minval, float maxval,int maxPixels)
{

  float eval=val<maxval?val:maxval;
  eval=eval>minval?eval:minval;
  
  return round(maxPixels/(maxval-minval)*(val-minval));
}


int statusDisplay(TFT_eSprite &m )
{
  int xpos =  0;
  int ypos =  0;

  String s, s2;
  //m.setFreeFont(FM18);
  //m.setFreeFont(&Monospaced_plain_16);
  m.setFreeFont(&Dialog_plain_32);
  
  return ypos;
}




void spiDisplayTask( void * parameter ) {
  int refresh = 0;
  unsigned int dispTimeout = 0;
  int bt = 0;
  int b;
  int dispmode = 0;
  unsigned int n = 0;
  int espBatV;
  displayInit();
  display.fillScreen(CUSTOM_DARK);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(0, 0);
  display.println("spiDisplayTask");
  Serial.println("spiDisplayTask:Initialized\n");
 // display.setFreeFont(FM12);
  while (1) {
    b = digitalRead(BUTTON);
    
    if (b == 0 && bt == 1) {
      refresh = 1;
      dispTimeout = n;
    }
    bt = b;
    // display.fillScreen(CUSTOM_DARK);
    TFT_eSprite m = TFT_eSprite(&display);
    m.setColorDepth(8);
    m.createSprite(display.width(), 140);
    m.fillSprite(TFT_BLACK);

    //m.setTextFont(2);
    m.setFreeFont(FM12);
    m.setTextDatum(TL_DATUM);
    m.setTextColor(TFT_WHITE);
    m.setCursor(0, 0);
    m.println("b="+String(b));
    m.println();

    if (bt) statusDisplay(m);
    else networkDisplay(m);
    m.pushSprite(0, 0);
    m.deleteSprite();

    m.createSprite(display.width(), 128);
    m.fillSprite(TFT_BLACK);
    m.pushSprite(0, 200);
    m.fillSprite(TFT_BLUE);
    delay(200);
  }

}


#endif
