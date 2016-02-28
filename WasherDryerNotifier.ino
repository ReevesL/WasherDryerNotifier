#define WASHER  1   // washing machine connected, 0==no
#define DRYER   1   // dryer connected, 0==no

#include "wifiSettings.h"



// Adafruit CC3000 wifi setup
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

//#define WLAN_SSID       "MySSID"           // cannot be longer than 32 characters!
//#define WLAN_PASS       "MyWLAnPass"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

// What page to grab!
#define WEBSITE      "www.adafruit.com"
#define WEBPAGE      "/testwifi/index.html"


/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/

uint32_t ip;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Setup");


}

void loop() {
  // put your main code here, to run repeatedly:

  if (WASHER)
  {
    monitorWasher();
  }

  if (DRYER)
  {
    monitorDryer();
  }

  // Temp wait to slow down the loop
  delay(1000000);

} //loop

void monitorWasher()
{
  int thisMachineType = 1; //Washer
  Serial.println ("Checking the washer");
  if (notifyIfttt(thisMachineType)) {
    Serial.println ("Wash notification good!");

  } else {
    Serial.println ("Notification fail!");
  }
  return;

}

void monitorDryer()
{ int thisMachineType = 2; //Dryer

  Serial.println ("Checking the dryer");
  if (notifyIfttt(thisMachineType)) {
    Serial.println ("Dryer notification good!");
  } else {
    Serial.println ("Notification fail!");
  }
  return;

}


int notifyIfttt (int machineType) {
  if (machineType == 1) // Washer
  {
    Serial.println ("Wash is done");
    return 1;
  } else if (machineType == 2)// Dryer
  {
    Serial.println ("Dryer is done");
    Serial.println ("But notification failed");
    return 0;

  }


}
