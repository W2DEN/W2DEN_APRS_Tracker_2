/*
* This is W2DEN's attempt at a Teensy3.1 APRS Tracker.
* This is V 2 adding a menu system to allow user input.
*
* The bases for this sketch comes from Richard Nash (KC3ARY). Without his code I'd still be searching
* Also thanks to:
*
    Ben Buxton: rotary encoder code ( http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html )
    9W2SVT: ASPRS Arduino with Display ( http://9w2svt.blogspot.com/ )
    M1GEO: APRS Blog ( http://www.george-smart.co.uk/wiki/APRS )
    KI4MCW: Balloonery ( https://sites.google.com/site/ki4mcw/ )
    BeRTOS Project ...
    Jinseok Jeon (JeonLab.wordpress.com): UTC calculator
    The APRS libraries come from the Trackduino project with modes by many before it got here.
 *
   */
#define thisver "2.00" ////////////////////////////////// VERSION

// includes
#include <WProgram.h>
#include <GPS.h>
#include <aprs.h>
#include <EEPROM.h>

// Set up the display
#include "SPI.h"
#include "ILI9341_t3.h"
ILI9341_t3 tft = ILI9341_t3(10, 9, 8, 11, 14, 12);
static const int line = 25; //# of lines on screen @ font size 3

// rotary selector
#include <Bounce.h>
#define ROTARY_PIN1 5
#define ROTARY_PIN2 6
#define BUTTONPIN   4
#define DIR_CCW 0x10
#define DIR_CW 0x20
Bounce pushbutton = Bounce(BUTTONPIN, 10);
// Use the full-step state table (emits a code at 00 only)
const unsigned char ttable[7][4] = {
  {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x10},
  {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
  {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x20},
  {0x6, 0x5, 0x4,  0x0},
};
volatile unsigned char state = 0;
void rotary_init() { // setas up the rotatry pins
  pinMode(ROTARY_PIN1, INPUT_PULLUP);
  pinMode(ROTARY_PIN2, INPUT_PULLUP);
  pinMode(BUTTONPIN, INPUT_PULLUP);
}
/* Read input pins and process for events. Call this either from a
 * loop or an interrupt (eg pin change or timer).
 * Returns 0 on no event, otherwise 0x80 or 0x40 depending on the direction.
 */
unsigned char rotary_process() {
  unsigned char pinstate = (digitalRead(ROTARY_PIN2) << 1) | digitalRead(ROTARY_PIN1);
  state = ttable[state & 0xf][pinstate];
  return (state & 0x30);
}
///////////////////////////////////////////////////////////// this defines the xmit time interval
int dTime = 0.25 * 60 * 1000; //Minutes *  seconds * milliseconds

//This is for the UTC date correction//////////////////////////////////////////////
int DSTbegin[] = { //DST 2013 - 2025 in Canada and US
  310, 309, 308, 313, 312, 311, 310, 308, 314, 313, 312, 310, 309
};
int DSTend[] = { //DST 2013 - 2025 in Canada and US
  1103, 1102, 1101, 1106, 1105, 1104, 1103, 1101, 1107, 1106, 1105, 1103, 1102
};
int DaysAMonth[] = { //number of days a month
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

int gpsYear;
int gpsMonth;
int gpsDay = 0;
int gpsHour;
int gpsSpeed;
#define knotToMPH 1.15078 // GPS speed is in knots... this is the conversion
////////////////////////////////////////////////////////////////////////////////

// Define the I/O pins
#define PTT_PIN 13 // Push to talk pin
#define ROTARY_PIN1 5
#define ROTARY_PIN2 6
#define BUTTONPIN   4

////////////// user data set up //////////////////////////////
//
//this is all new. User data are read from EEPROM
//
// create the struct instance for the calls and paths//////////
//
struct PathAddress addresses[] = {
  {(char *)NULL, 0},  // Destination callsign
  {(char *)NULL, 0},  // Source callsign
  {(char *)NULL, 0},  // Digi1 (first digi in the chain)
  {(char *)NULL, 0}   // Digi2 (second digi in the chain)
};
//
// set up variables to hold EEPROM data //////////////////////
// make these global so we can edit them.
// variables are loaded to each xmit
//
int8_t TimeZone;
char sCall[10];     // holds the s source call
char dCall[10];     // holds the d destination call
char buf;           // buffer for EEPROM read
char symTable;      // symbole table \ or //
char symbol;        // symbol > car etc/
char myComment[36]; // comments holder
///////////////////////////////////////////////////////////////

HardwareSerial &gpsSerial = Serial1;
GPS gps(&gpsSerial, true);
////////////////////////////////////////////////////////// setup()
void setup()
{
  Serial.begin(9600); // For debugging output over the USB port
  // read the EEPROM user data into memory /// will be updated///////
  TimeZone = EEPROM.read(1);
  for (int i = 2; i < 11; i++) { // Source call
    buf = EEPROM.read(i);
    if (buf == 32) {
      break;
    }
    sCall[i - 2] = buf;
  }
  for (int i = 12; i < 21; i++) { //destinationn call
    buf = EEPROM.read(i);
    if (buf == 32) {
      break;
    }
    dCall[i - 12] = buf;
  }

  for (int i = 24; i < 58; i++) { // comment
    buf = EEPROM.read(i);
    myComment[i - 24] = buf;
  }


  Serial.print(myComment);
  Serial.println(":OK");
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(0);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(0, 50);
  tft.println("W2DEN's");
  tft.println("APRS");
  tft.println("Tracker");
  tft.println("Loading ...");

  gps.startSerial(9600);
  delay(1000);
  gps.setSentencesToReceive(OUTPUT_RMC_GGA);
  rotary_init(); // initialize the rotary
  // Set up the APRS module
  aprs_setup(50, // number of preamble flags to send
             PTT_PIN, // Use PTT pin
             500, // ms to wait after PTT to transmit
             0, 0 // No VOX ton
            );

  while (!(gps.sentenceAvailable())) {
    delay(1000);
  }
  gps.parseSentence();
  //Serial.print("APRS Initial");
  //Serial.printf("Location: %f, %f altitude %f\n\r", gps.latitude, gps.longitude, gps.altitude);
  gps.dataRead();
  broadcastLocation(gps, myComment);
  gpsSpeed = gps.speed;
  tft.fillScreen(ILI9341_BLACK);
  display();

}

/////////////////////////////////////////////////////////////////////// broadcastLocation()
//Function to broadcast your location
void broadcastLocation(GPS &gps, const char *bcomment)
{
  // If above 5000 feet switch to a single hop path
  int nAddresses;
  // load the user data variables into the addresses[] array for xmit
  //
  addresses[0].callsign = dCall;
  addresses[0].ssid = EEPROM.read(21);
  addresses[1].callsign = sCall;
  addresses[1].ssid = EEPROM.read(11);
  symbol =  EEPROM.read(23);
  symTable =  EEPROM.read(22);
  if (gps.altitude > 1500) {
    // APRS recomendations for > 5000 feet is:
    // Path: WIDE2-1 is acceptable, but no path is preferred.
    nAddresses = 3;
    addresses[2].callsign = "WIDE2";
    addresses[2].ssid = 1;
  } else {
    // Below 1500 meters use a much more generous path (assuming a mobile station)
    // Path is "WIDE1-1,WIDE2-2"
    nAddresses = 4;
    addresses[2].callsign = "WIDE1";
    addresses[2].ssid = 1;
    addresses[3].callsign = "WIDE2";
    addresses[3].ssid = 2;
  }
  // For debugging print out the path
  Serial.print("APRS(");
  Serial.print(nAddresses);
  Serial.print("): ");
  for (int i = 0; i < nAddresses; i++) {
    Serial.print(addresses[i].callsign);
    Serial.print('-');
    Serial.print(addresses[i].ssid);
    if (i < nAddresses - 1)
      Serial.print(',');
  }
  Serial.print(' ');
  Serial.print(symTable);
  Serial.print(symbol);
  Serial.println();


  // Send the packet
  aprs_send(addresses, nAddresses
            , gps.day, gps.hour, gps.minute
            , gps.latitude, gps.longitude // degrees
            , gps.altitude // meters
            , gps.heading
            , gps.speed
            , symTable //SYMBOL_TABLE
            , symbol //SYMBOL_CHAR
            , bcomment);
  //Serial.print("APRS sent");
  //Serial.printf("Location: %f, %f altitude %f\n\r", gps.latitude, gps.longitude, gps.altitude);
}

uint32_t timeOfAPRS = 0;
bool gotGPS = false;
// the loop() method runs over and over again,
// as long as the board has power

////////////////////////////////////////////////////////////////////// loop()
void loop()
{
  // capture the button press
  if (pushbutton.update()) {
    if (pushbutton.fallingEdge()) {
      menu();
    }
  }
  // get GPS data
  if (gps.sentenceAvailable()) gps.parseSentence();
  if (gps.newValuesSinceDataRead()) {
    gotGPS = true; // @TODO: Should really check to see if the location data is still valid
    gps.dataRead();
    //Serial.printf("Location: %f, %f Knots: %f\n\r", gps.latitude, gps.longitude, gps.speed);
    //Serial.printf("dTime %i timeOf %i millis %i\n\r",dTime,timeOfAPRS,millis());
    displayCountDown();
  }
  // do we xmit?
  if (gotGPS && timeOfAPRS + dTime < millis()) {
    broadcastLocation(gps, myComment );
    timeOfAPRS = millis();
    display();
  }
}
/////////////////////////////////////////////////////////////////////////////
//       FUNCTIONS from here down
//////////////////////////////////////////////////////////// menu()
void menu()
{
  // lets set it up
  int mStart = 0;   //First menu item
  int mEnd   = 2;   // last menu item
  int mPick  = 0;   // menu choice
  int mB4    = 0;    // line # befor move
  //this defines the menu
  String menu1[] = {"Return", "Send/Return", "UTC Offset"};
  //now draw it
  tft.fillScreen(ILI9341_BLUE);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
  tft.setCursor(0, 0);
  tft.println("Menu");
  tft.setCursor(160, 0);
  tft.print("  ");
  tft.setCursor(160, 0);
  tft.print(mPick);
  for (int i = mStart; i <= mEnd; i++) {
    tft.setCursor(0, (i + 1) * 25);
    if (i == mPick) {
      tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
    }
    else
    {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    tft.print(String(i) + " ");
    tft.print(menu1[i]);
  }
  // now loop looking for a rotation or a button press
  while (true)
  {
    unsigned char result = rotary_process(); // rotated?
    if (result) {                            // the button was rotated
      if (result == DIR_CCW) {               // highlight the next choice
        if (--mPick < mStart) mPick = mEnd;
      }
      else {
        if (++mPick > mEnd) mPick = mStart;
      }
      tft.setCursor(160, 0);                 // draw it
      tft.print("  ");
      tft.setCursor(160, 0);
      tft.print(mPick);
      Serial.printf("mPick: %i, mB4: %i\n\r", mPick, mB4);
      tft.setCursor(0, (mB4 + 1) * 25);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      tft.print(String(mB4) + " ");
      tft.print(menu1[mB4]);
      tft.setCursor(0, (mPick + 1) * 25);
      tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
      tft.print(String(mPick) + " ");
      tft.print(menu1[mPick]);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      mB4 = mPick;
    }
    if (pushbutton.update()) {                // button pushed
      if (pushbutton.fallingEdge()) {
        switch (mPick) {                      //handle the button push
          case 0:  // return
            tft.fillScreen(ILI9341_BLACK);
            display();
            return;
          case 1:  // send and return
            gps.dataRead();
            broadcastLocation(gps, myComment );
            tft.fillScreen(ILI9341_BLACK);
            timeOfAPRS = millis();
            display();
            return;
          case 2: // utc offset set
            int lTZ = -12;      // lower TZ limit
            int uTZ = 14;       // upper TZ limit
            int nTZ = TimeZone; // new TZ
            tft.fillScreen(ILI9341_BLUE);
            tft.setTextSize(3);
            tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
            tft.setCursor(0, 0);
            tft.println("UTC Offset");
            tft.setCursor(0, 25);
            tft.print("Now: ");
            tft.print(TimeZone);
            tft.setCursor(0, 75);
            tft.print("New: ");
            tft.setCursor(0, 125);
            tft.setTextSize(2);
            tft.println("Rotate to Time Zone");
            tft.println("Push to accept");
            tft.println("");
            tft.println("Standard time in USA");
            tft.println("DST is automaic");
            tft.setTextSize(3);
            tft.setCursor(100, 75);
            tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
            tft.print(nTZ);
            while (true) {
              if (pushbutton.update()) {                // button pushed
                if (pushbutton.fallingEdge()) {
                  if (TimeZone != nTZ) {
                    TimeZone = nTZ;
                    EEPROM.update(1, TimeZone);
                  }

                  tft.fillScreen(ILI9341_BLACK);
                  display();
                  return;
                }
              }
              unsigned char result = rotary_process(); // rotated?

              if (result) {
                if (result == DIR_CCW) {
                  nTZ--;
                  if (nTZ < lTZ) {
                    nTZ = uTZ;
                  }
                }
                else {
                  nTZ++;
                  if (nTZ > uTZ) {
                    nTZ = lTZ;
                  }
                }
                //tft.setTextColor(ILI9341_WHITE,ILI9341_BLUE);
                //tft.setCursor(0, 75);
                //tft.print("New: ");
                tft.setCursor(100, 75);
                tft.print("   ");
                tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
                tft.setCursor(100, 75);
                tft.print(nTZ);
              }
            }




            delay(10000);
            tft.fillScreen(ILI9341_BLACK);


            display();
            return;

        }
      }
    }
  }

}
//////////////////////////////////////////////////////////// displayCountDown()
void displayCountDown()
{
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(190, 0);
  tft.print("    ");
  tft.setCursor(190, 0);
  tft.print((dTime - (millis() - timeOfAPRS)) / 1000 );
}
///////////////////////////////////////////////////////////////// display()
void display()
{
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(0, 0);
  gpsYear  = gps.year;
  gpsMonth = gps.month;
  gpsDay   = gps.day;
  gpsHour  = gps.hour;
  gpsHour += TimeZone; // Time zone correction
  // DST fix
  if (gpsMonth * 100 + gpsDay >= DSTbegin[gpsYear - 13] &&
      gpsMonth * 100 + gpsDay < DSTend[gpsYear - 13]) gpsHour += 1;
  if (gpsHour < 0)
  {
    gpsHour += 24;
    gpsDay -= 1;
    if (gpsDay < 1)
    {
      if (gpsMonth == 1)
      {
        gpsMonth = 12;
        gpsYear -= 1;
      }
      else
      {
        gpsMonth -= 1;
      }
      gpsDay = DaysAMonth[gpsMonth - 1];
    }
  }
  if (gpsHour >= 24)
  {
    gpsHour -= 24;
    gpsDay += 1;
    if (gpsDay > DaysAMonth[gpsMonth - 1])
    {
      gpsDay = 1;
      gpsMonth += 1;
      if (gpsMonth > 12) gpsYear += 1;
    }
  }
  char sz[32];
  sprintf(sz, "%02d/%02d/%02d ", gpsMonth, gpsDay, gpsYear);
  tft.println(sz);
  //char sz[32];
  sprintf(sz, "%02d:%02d ", gpsHour, gps.minute);
  tft.println(sz);

  tft.setTextSize(3);
  tft.setCursor(0, line * 3);
  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  displayLatLong(gps.latitude);

  tft.setTextSize(2);
  if (gps.latitude < 0)
  {
    tft.println(" S");
  }
  else
  {
    tft.println(" N");
  }
  tft.setTextSize(3);
  tft.setCursor(0, line * 5);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  displayLatLong(gps.longitude);
  tft.setTextSize(2);
  if (gps.longitude < 0)
  {
    tft.println(" W");
  }
  else
  {
    tft.println(" E");
  }
  tft.setTextSize(3);
  tft.setCursor(0, line * 7);
  tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
  tft.setTextSize(4);
  tft.print("          ");
  tft.setCursor(0, line * 7);
  printStr(String(int(gps.speed * knotToMPH) ), 5, false);
  tft.setTextSize(3);
  printStr(" mph", 4, true);
  tft.setCursor(0, line * 9);
  tft.setTextSize(4);
  tft.print("          ");
  tft.setTextSize(3);
  tft.setCursor(0, line * 9);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  printStr(gps.heading, 4, false);
  printStr(" deg", 4, true);
  tft.setCursor(0, 280);
  tft.setTextSize(2);
  printStr("Sats:", 6, false);
  printStr(String(gps.satellites), 2, false);
  tft.setCursor(150, 280);
  float x = analogRead(39);
  if (x <= 1500) // approximately 3 vdc
  {
    tft.setTextColor(ILI9341_GREEN);
    tft.print("V+ OK");
  }
  else
  {
    tft.setTextColor(ILI9341_RED);
    tft.print("V+ ");
    tft.println(( (178 * x * x + 2688757565 - 1184375 * x)  / 372346 ) / 1000);
  }
  tft.setCursor(170, 300);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(1);
  printStr(thisver, 6, false); // display version

}
/////////////////////////////////////////////////////////////// displayLatLong()
static void displayLatLong(float val)
// converts decimal degrees to degrees and decimal minutes (APRS format)
{
  float wlong = val;
  char charVal[10];
  int deglong = wlong; //long;
  wlong -= deglong; // remove the degrees from the calculation
  wlong *= 60; // convert to minutes
  float mlong = wlong;
  String pDegrees = " " + String(abs(deglong));
  //return pDegrees;
  printStr(pDegrees, 4, false);
  tft.setTextSize(2);
  tft.print(char(247));
  tft.setTextSize(3);
  mlong = abs(mlong);
  dtostrf(mlong, 6, 4, charVal);
  pDegrees = "";

  for (unsigned int i = 0; i < sizeof(charVal); i++) // was sizeof
  {
    pDegrees += charVal[i];
  }
  if (mlong < 10)
  {
    printStr(" ", 2, false);
    printStr(pDegrees, 5, false);
  }
  else
  {
    printStr(" ", 1, false);
    printStr(pDegrees, 6, false);
  }
}

///////////////////////////////////////////////////////////////// printStr()
static void printStr(String istring, unsigned int len, boolean rtn)
{

  String sout = "";
  unsigned int slen = istring.length();// this how long it is
  istring = istring.trim();
  if (slen > len)
  {
    sout = istring.substring(0, len);
  }
  else
  {
    sout = istring;
    while (sout.length() < len)
    {
      sout = " " + sout;
    }
  }
  if (rtn)
  {
    tft.println(sout);
  }
  else
  {
    tft.print(sout);
  }
}




