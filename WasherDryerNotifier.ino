// A separate file to hold passwords and keys which doesn't get checked in
// See privateSettings_example.h for contents
#include "privateSettings.h"

#include <SoftwareSerial.h>
//#include <PString.h>
//#include <Streaming.h>
#include <Arduino.h>



// Debugging flags
#define _DEBUG  1   // try to push data to serial monitor (save memory by setting to 0)
#define NOTWEET 0   // don't start networking or send tweet 
#define WEBMSG  0   // pull tweet strings from the web, 0==no

#define WASHER  1   // washing machine connected, 0==no
#define DRYER   1   // dryer connected, 0==no


// Dryer settings
// Using analog 0 for the Current Transformer
#define CTPIN_D              0
#define WAITONTIME         15000        //In milliseconds, time before setting appliace as "running" (used for Washer & Dryer)
#define WAITOFFTIME        30000        //In milliseconds, time to wait to make sure dryer is really off (avoid false positives)
#define CURRENT_THRESHOLD    20.0    //In AMPS
double CURRENT_OFFSET  =   0.0;  // Dryer offset (made a double so it can auto set)
#define NUMADCSAMPLES      4000     //Number of samples to get before calculating RMS

// Washer settings
// Using analog 5 for the Current Transformer for Washer
#define CTPIN_W            5
#define CURRENT_THRESHOLD_W      10.0    // In AMPS
double CURRENT_OFFSET_W   =     0.0;  // Washer offset in Amps
#define WAITOFFTIME_W        99000    // In MS 

//STATUS LEDS
#define DRYLED   5     //Dryer status
#define WASHLED    7   //Washer status
#define REDLED     6   //General status/error

//If using 3.3V Arduino use this
#define ADC_OFFSET 1.65                    //Resistor divider is used to bias current transformer 3.3V/2 = 1.65V
#define ADCVOLTSPERDIVISION 0.003222656    //3.3V/1024 units

//If using 5V Arduino use this
//#define ADC_OFFSET 2.5                   //Resistor divider is used to bias current transformer 5V/2 = 2.5V
//#define ADCVOLTSPERDIVISION 0.0047363    //5V / 1024 units

//These are the various states during monitoring. WAITON and WAITOFF both have delays
//that can be set with the WAITONTIME and WAITOFFTIME parameters
enum STATE {
  WAITON,    //Washer/Dryer is off, waiting for it to turn on
  RUNNING,   //Exceeded current threshold, waiting for washer/dryer to turn off
  WAITOFF    //Washer/dryer is off, sends notifications, and returns to WAITON
};

//This keeps up with the current state of the FSM, default startup setting is the first state of waiting for the dryer to turn on.
STATE dryerState = WAITON;
STATE washerState = WAITON;

boolean printCurrent = true;



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


//defined these in privateSettings.h so my stuff won't get checked into git (in .gitignore)
//#define WLAN_SSID       "MySSID"           // cannot be longer than 32 characters!
//#define WLAN_PASS       "MyWLAnPass"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
// received before closing the connection.  If you know the server
// you're accessing is quick to respond, you can reduce this value.

//defined these in privateSettings.h so my stuff won't get checked into git (in .gitignore)
// What page to grab!
//#define WEBSITE      "www.adafruit.com"
//#define WEBPAGE      "/testwifi/index.html"


/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/

uint32_t ip;

// Buffer for reading messages?
char strBuf[165];      
//String str(strBuf, sizeof(strBuf));
#define TEMP_MSG_BUFFER_SIZE 100
char tempMsgBuffer[TEMP_MSG_BUFFER_SIZE];

void setup() {
  Serial.println("Setup"); // setup starting

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // Configure LEDs
  pinMode(DRYLED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(WASHLED, OUTPUT);

  // Turn on the red LED during setup
  digitalWrite(REDLED, HIGH);

  if (_DEBUG) {
    Serial.println("Washer Dryer Tweeter");
    Serial.println("==========================");

  }


  if (!NOTWEET) { // check to see if debugging/no tweeting
    if (_DEBUG) {
      Serial.println("Initializing network");

    }

    // Start wifi


    Serial.println(F("Hello, CC3000!\n"));

    Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);

    /* Initialise the module */
    Serial.println(F("\nInitializing..."));
    if (!cc3000.begin())
    {
      Serial.println(F("Couldn't begin()! Check your wiring?"));
      while (1);
    }

    // Optional SSID scan
    // listSSIDResults();

    Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
    if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
      Serial.println(F("Failed!"));
      while (1);
    }

    Serial.println(F("Connected!"));

    /* Wait for DHCP to complete */
    Serial.println(F("Request DHCP"));
    while (!cc3000.checkDHCP())
    {
      delay(100); // ToDo: Insert a DHCP timeout!
    }

    /* Display the IP address DNS, Gateway, etc. */
    //  while (! displayConnectionDetails()) {
    //    delay(1000);
    //  }

  }
  // Instead of having the offset hard coded, I've made it zero itself
  // on boot. This removed variance in the offset that seemed to come
  // from different power sources for the Arduino.
  //
  // Pull current on startup to zero the measurements
  double theMeasurement = 0;
  // Pull washer
  for (int i = 0; i < 3; i++) {
    theMeasurement += takeCurrentMeasurement(CTPIN_W);
  }
  //
  CURRENT_OFFSET_W = 0 - (theMeasurement / 3);

  // display the calculation
  if (_DEBUG) {
    Serial.print  ("Washer offset: ");
    Serial.print (CURRENT_OFFSET_W);
    Serial.print ("\r\n");
    delay(5000);
  }


  theMeasurement = 0; // zero the measurement variable
  // Pull dryer
  for (int i = 0; i < 3; i++) {
    theMeasurement += takeCurrentMeasurement(CTPIN_D);
  }
  //
  CURRENT_OFFSET = 0 - (theMeasurement / 3);

  // display the calculation
  if (_DEBUG) {
    Serial.print ("Dryer offset: ");
    Serial.print (CURRENT_OFFSET);
    Serial.print ("\r\n");
    delay(5000);
  }
  // Setup successful, turn out red LED, turn on the wash and dry LEDs
  digitalWrite(REDLED, LOW);
  digitalWrite(WASHLED, HIGH);
  digitalWrite(DRYLED, HIGH);

}// end setup

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

void monitorDryer() {

  int thisMachineType = 2; //Dryer

  Serial.println ("Checking the dryer");
  // old check dryer code from previous app
  static unsigned long loopCount = 0;
  static unsigned long startTime;
  static unsigned long runStartTime;
  static boolean waitCheck;
  double current;

  loopCount++;
  switch (dryerState)
  {
    case WAITON:
      current = takeCurrentMeasurement(CTPIN_D);
      if (printCurrent)
      {
        Serial.print ("Dryer Current: ");
        Serial.print (current);
        Serial.print (" Dryer Threshold: ");
        Serial.print (CURRENT_THRESHOLD);
        Serial.print ("\r\n");
      }
      if (current > CURRENT_THRESHOLD)
      {
        if (!waitCheck)
        {
          startTime = millis();
          waitCheck = true;
          Serial.print ("Started watching dryer in WAITON. startTime: ");
          Serial.print (startTime);
          Serial.print ("\r\n");
        }
        else
        {
          if ( (millis() - startTime  ) >= WAITONTIME)
          {
            Serial.println ("Changing dryer state to RUNNING");
            dryerState = RUNNING;
            waitCheck = false;
            runStartTime = millis();
          }
        }
      }
      else
      {
        if (waitCheck)
        {
          Serial.println ("False alarm, not enough time elapsed to advance to running");
        }
        waitCheck = false;
      }
      break;

    case RUNNING:
      current = takeCurrentMeasurement(CTPIN_D);
      digitalWrite(DRYLED, !digitalRead(DRYLED));
      if (printCurrent)
      {
        Serial.print ("Dryer Current: ");
        Serial.print (current);
       Serial.print (" Threshold: ");
       Serial.print (CURRENT_THRESHOLD);
                Serial.print ("\r\n");
      }
      if (current < CURRENT_THRESHOLD)
      {
        Serial.println ("Changing dryer state to WAITOFF");
        dryerState = WAITOFF;
      }
      break;

    case WAITOFF:
      if (!waitCheck)
      {
        startTime = millis();
        waitCheck = true;
        Serial.print ("Entered WAITOFF. startTime: ");
        Serial.print (startTime);
        Serial.print ("\r\n");


        // setting the RED LED to solid on while waiting to send off signal.
        digitalWrite(REDLED, HIGH);
      }

      current = takeCurrentMeasurement(CTPIN_D);

      if (current > CURRENT_THRESHOLD)
      {
        Serial.println ("False Alarm, dryer not off long enough, back to RUNNING we go!");
        waitCheck = false;
        dryerState = RUNNING;
      }
      else
      {
        if ( (millis() - startTime) >= WAITOFFTIME)
        {
          Serial.println ("Dryer cycle complete");
//          str.begin();

          //        str.print(GetAMessage('d'));

          // How long was the cycle?
          // I don't really care, so commeting out
          // Uncomment if you want this info in the tweet
          //        str.print( F(" Run Time: " ));
          //        double totalRun = (double) (millis() - runStartTime) / 1000 / 3600;
          //        str.print(totalRun);
          //        if(totalRun < 0)
          //          str.print("hr");
          //        else
          //          str.print("hrs");

          // @mention my account so I get a push notification
          // TODO: Should move this into a common settings area
          //        to make it easier to configure.
          //str.print(" @YOUR_TWITTER_HANDLE_HERE");

          //        Serial << F("Posting to twitter") << endl;
          //        Serial << F("Posting the following: ") << endl;
          //        Serial << str << endl;

          //Tweet it (unless in notweet debugging mode)
          if (!NOTWEET) { // check to see if debugging/no tweeting
            // new notification stub
            if (notifyIfttt(thisMachineType)) {
              Serial.println ("Dryer notification good!");
            } else {
              Serial.println ("Notification fail!");
            }

          }
          else {
            Serial.println ("----- Not posted: Tweets off (debug). -----");

          }



          Serial.println ("Resetting state back to WAITON");
          dryerState = WAITON;
          waitCheck = false;
          digitalWrite(DRYLED, HIGH);
          digitalWrite(REDLED, LOW);
        }
      }
  }


  return;

}


int notifyIfttt (int machineType) {
  if (machineType == 1) // Washer
  {
    Serial.println ("Wash is done");

    // doing a connection test here

    ip = 0;
    // Try looking up the website's IP address
    Serial.print(WEBSITE); Serial.print(F(" -> "));
    while (ip == 0) {
      if (! cc3000.getHostByName(WEBSITE, &ip)) {
        Serial.println(F("Couldn't resolve!"));
      }
      delay(500);
    }

    cc3000.printIPdotsRev(ip);

    // Optional: Do a ping test on the website
    /*
      Serial.print(F("\n\rPinging ")); cc3000.printIPdotsRev(ip); Serial.print("...");
      replies = cc3000.ping(ip, 5);
      Serial.print(replies); Serial.println(F(" replies"));
    */

    /* Try connecting to the website.
       Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
    */
    Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
    if (www.connected()) {
      www.fastrprint(F("GET "));
      www.fastrprint(WEBPAGE);
      www.fastrprint(F(" HTTP/1.1\r\n"));
      www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
      www.fastrprint(F("\r\n"));
      www.println();
    } else {
      Serial.println(F("Connection failed"));
      return 0;
    }

    Serial.println(F("-------------------------------------"));

    /* Read data until either the connection is closed, or the idle timeout is reached. */
    unsigned long lastRead = millis();
    while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
      while (www.available()) {
        char c = www.read();
        Serial.print(c);
        lastRead = millis();
      }
    }
    www.close();
    Serial.println(F("-------------------------------------"));

    /* You need to make sure to clean up after yourself or the CC3000 can freak out */
    /* the next time your try to connect ... */
    Serial.println(F("\n\nDisconnecting"));
    cc3000.disconnect();


    return 1;
  } else if (machineType == 2)// Dryer
  {
    Serial.println ("Dryer is done");
    Serial.println ("But notification failed");
    return 0;

  }


}

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;

  if (!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

// measures current
double takeCurrentMeasurement(int channel)
{
  static double VDOffset = 1.65;

  //Equation of the line calibration values
  //double factorA = 15.2; //factorA = CT reduction factor / rsens
  //double factorA = 25.7488;
  double factorA = 33.0113;
  //double factorA = 29.03225;
  double Ioffset =  -0.04;
  double theCurrentOffset = 0.0;

  if (channel == CTPIN_D) { // check if washer or dryer
    theCurrentOffset = CURRENT_OFFSET; // use dryer offset
  }
  else {
    theCurrentOffset = CURRENT_OFFSET_W; // use washer offset
  }

  //Used for calculating real, apparent power, Irms and Vrms.
  double sumI = 0.0;

  int sum1i = 0;
  double sumVadc = 0.0;

  double Vadc, Vsens, Isens, Imains, sqI, Irms;

  for (int i = 0; i < NUMADCSAMPLES; i++)
  {
    int val = 0;
    val = analogRead(channel);
    Vadc = val * ADCVOLTSPERDIVISION;
    Vsens = Vadc - VDOffset;
    //Current transformer scale to find Imains
    Imains = Vsens;
    //Calculates Voltage divider offset.
    sum1i++;
    sumVadc = sumVadc + Vadc;
    if (sum1i >= 1000) {
      VDOffset = sumVadc / sum1i;
      sum1i = 0;
      sumVadc = 0.0;
    }

    //Root-mean-square method current
    //1) square current values
    sqI = Imains * Imains;
    //2) sum
    sumI = sumI + sqI;
  }
  Irms = factorA * sqrt(sumI / NUMADCSAMPLES) + theCurrentOffset;
  sumI = 0.0;
  return Irms;
}
