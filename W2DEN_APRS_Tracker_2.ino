/*
* This is W2DEN's attempt at a Teensy3.1 APRS Tracker.
* This is V 2.02 adding a menu system to allow user input.
* see N4SER.org for details
*
* Compiler Macro Substitutes (#define) in ALL_CAPS with underline for spaces
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
********************************************************************
*
*/
#define thisver "2.02" ////////////////////////////////// VERSION
// these define the starting EEPROM addresses.
// easier to change these than dig for the constants.
#define UTC_OFFSET 1
#define XMIT_TIME  2
#define MY_CALL    4
#define MY_SSID   10
#define DEST_CALL 11
#define DEST_SSID 17
#define SYM_TABLE 18
#define SYMBOL    19
#define COMMENT   20
#define SB_ENABLE    55 // these are SmartBeaconing eePROM addresses
#define SBFAST_SPEED 56
#define SBFAST_RATE  58
#define SBSLOW_SPEED 60
#define SBSLOW_RATE  62
#define SBTURN_TIME  64
#define SBTURN_ANGLE 66
#define SBTURN_SLOPE 68
#define AXDELAY      70
#define AXFLAGS      72
#define AXVOXON      74
#define AXVOXSILENT  76
#define PTT_PIN      78
#define TFT_ONOFF    80 // tft On / Off (1 / 0) EEPROM address only 1 byte
#define SQUELCH      81 // the squelch threshol eeprom address 

#define COMMLENGTH  35 // length of the AX.25 comment
#define NUM_SYMBOLS 10 // number of symbols in the symbols[] table

// includes
#include <WProgram.h>
#include <GPS.h>
#include <aprs.h>
#include <EEPROMex.h> // expanded EEPROM library

#include "SPI.h" // Set up the display
#include "ILI9341_t3.h"
ILI9341_t3 tft = ILI9341_t3(10, 9, 8, 11, 14, 12); //(CS,DC,RST,MOSI,SCLK,MISC)

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
int DaysAMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};//number of days a month
int gpsYear;
int gpsMonth;
int gpsDay = 0;
int gpsHour;
#define knotToMPH 1.15078 // GPS speed is in knots... this is the conversion
////////////////////////////////////////////////////////////////////////////////

// Define the I/O pins
//#define PTT_PIN 13 // Push to talk pin tis is now eePROM
#define ROTARY_PIN1   5
#define ROTARY_PIN2   6
#define BUTTONPIN     4
#define TFT_ONOFF_PIN 3 // tft On Off PWM pin
#define SQUEL_PIN     A6       // these are constants for the squelch threshold
#define SQUEL_REF     INTERNAL
#define SQUEL_RES     10
#define SQUEL_AVE     16

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
int8_t TimeZone;    // utc offseti
uint16_t sTime;      // xmit delay if no smart beacon
char sCall[7];      // holds the s source call
char dCall[7];      // holds the d destination call
char buf;           // buffer for EEPROM read
char symTable;      // symbole table \ or //
char symbol;        // symbol > car etc/
char myComment[36]; // comments holder 35 chars + end
const char* symbols [][9] = { //table of symbols
  {"House", "/", "/"},
  {"Car", "/", ">"},
  {"M-Cycle", "/", "<"},
  {"Van", "/", "v"},
  {"Truck", "/", "k"},
  {"Bike", "/", "b"},
  {"Balloon", "/", "O"},
  {"SailBoat", "/", "y"},
  {"PwrBoat", "/", "s"},
  {"School", "/", "K"}
};
int8_t   sbEnable;       // use sb or not
uint16_t sbFastSpeed;    //mph speeds stored and compared as integers tostop oscilatting
uint16_t sbFastRate;     //seconds
uint16_t sbSlowSpeed;    // mph as integer
uint16_t sbSlowRate;
uint16_t sbMinTurnTime;  //sec
uint16_t sbMinTurnAngle; //degrees
uint16_t sbTurnSlope;    //
uint16_t axDelay;        // milliseconds
uint16_t axFlags;        // number of flags to send
uint16_t axVoxOn;        // mseconds vox tone sent to xmitter 0 for off
uint16_t axVoxSilent;    // mseconds VOX tone silent  0 for off
uint16_t pttPin;         // PTT Teensy Pin usually 13, 0 for off
int8_t   tftOnOff;       // tft brightness setting
uint16_t squelch;        // squelch threshold vlaue holder

uint16_t mySpeed   = 0;  // Holds gps.speed for processing
uint16_t myHeading = 0;
uint16_t sbSpeed   = 0;  //prior read speed
uint16_t sbHeading = 0;  // prior heading
//////////////////////// pre set up variables///////////////////////////
HardwareSerial &gpsSerial = Serial1;
GPS gps(&gpsSerial, true);
void mNumChoice(int eePromAddress, int8_t *variable, int lTZ, int uTZ,  int nTZ, String title, String help , uint16_t TTT = 0, uint16_t *variable2 = 0, bool e16bit = false );

uint32_t dTime;
uint32_t timeOfAPRS = 0;
bool gotGPS = false;

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// setup()
////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600); // For debugging output over the USB port
  // read the EEPROM user data into memory /// will be updated/////////
  /////////////////////////////////////////////////////////////////////
  TimeZone = EEPROM.read(UTC_OFFSET);
  // sTime is the start delay time sotred in eePROM as an int in seconds
  // dTime is the working xmit delay uint32_t starts as sTime then via SmartBeacon if activated.
  sTime = EEPROM.readInt(XMIT_TIME); // seconds
  dTime = sTime * 1000; // store the start time into dTime (delay) in milliseconds
  for (int i = MY_CALL; i < MY_CALL + 7; i++) { // Source call
    buf = EEPROM.read(i);
    if (buf == 32) break;
    sCall[i - MY_CALL] = buf;
  }
  for (int i = DEST_CALL; i < DEST_CALL + 7; i++) { //destinationn call
    buf = EEPROM.read(i);
    if (buf == 32) break;
    dCall[i - DEST_CALL] = buf;
  }

  for (int i = COMMENT; i < COMMENT + COMMLENGTH; i++) { // comment
    buf = EEPROM.read(i);
    myComment[i - COMMENT] = buf;
  }
  // now the SmartBeacon parameters...
  // 16 bit allows menu compatability
  sbEnable       = EEPROM.read(SB_ENABLE);
  sbFastSpeed    = EEPROM.readInt(SBFAST_SPEED); //mph speeds stored and compared as integers tostop oscilatting
  sbFastRate     = EEPROM.readInt(SBFAST_RATE);  //seconds
  sbSlowSpeed    = EEPROM.readInt(SBSLOW_SPEED); // mph as integer
  sbSlowRate     = EEPROM.readInt(SBSLOW_RATE);
  sbMinTurnTime  = EEPROM.readInt(SBTURN_TIME);  // sec
  sbMinTurnAngle = EEPROM.readInt(SBTURN_ANGLE); // degrees
  sbTurnSlope    = EEPROM.readInt(SBTURN_SLOPE); //
  axDelay        = EEPROM.readInt(AXDELAY);      // milliseconds
  axFlags        = EEPROM.readInt(AXFLAGS);      // number of flags to send
  axVoxOn        = EEPROM.readInt(AXVOXON );     // mseconds vox tone sent to xmitter 0 for off
  axVoxSilent    = EEPROM.readInt(AXVOXSILENT);  // mseconds VOX tone silent  0 for off
  pttPin         = EEPROM.readInt(PTT_PIN);      // PTT Teensy Pin usually 13, 0 for off
  tftOnOff       = EEPROM.read(TFT_ONOFF);       // tft On / Off: 1 or 0
  squelch        = EEPROM.readInt(SQUELCH);      // read ssquelch threshold from eePROM into variable
  tft.begin();
  pinMode(TFT_ONOFF_PIN, OUTPUT);
  digitalWrite(TFT_ONOFF_PIN, 1); /// this is the TFT display On Off set to on for boot
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(2);
  tft.setCursor(0, 250);
  tft.setTextSize(2);
  if (tftOnOff) {
    tft.print("Display: On");
  }
  else {
    tft.print("Display: Off");
  }
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(0, 50);
  tft.println("W2DEN's");
  tft.println("APRS");
  tft.println("Tracker");
  tft.println("Loading ...");

  // start the GPS polling and wait for valid data... may take 15+ seconds
  gps.startSerial(9600);
  gps.setSentencesToReceive(OUTPUT_RMC_GGA);
  rotary_init(); // initialize the rotary
  // wait for a GPS sentence
  while (!(gps.sentenceAvailable())) {
    delay(1000);
  }

  gps.parseSentence();
  gps.dataRead();
  //   only proced if the dates are valid
  char rot[9] = {'|', '/', (char)195, '\\', '|', '/', (char)195, '\\'};
  uint i = 0;
  while (gps.month <= 0 || gps.day <= 0 || gps.year <= 0) { // || gps.speed > 0) {
    Serial.print( gps.month);
    Serial.print( "/");
    Serial.print( gps.day);
    Serial.print( "/");
    Serial.println( gps.year);
    if (gps.sentenceAvailable()) gps.parseSentence();
    if (gps.newValuesSinceDataRead()) gps.dataRead();
    tft.setCursor(0, 150);
    tft.print(rot[i++]);
    if (i >= sizeof(rot) - 1) i = 0;
    delay(1000);
  }
  // Set up the APRS module
  aprs_setup(axFlags, // number of preamble flags (FLAG) (Hex: 0x7e, ASCII: ~, Binary: 01111110 )to send
             pttPin, // PTT Pin from eePROM
             axDelay, // ms to wait after PTT to transmit
             axVoxOn, axVoxSilent // VOX: tone length, silence length
            );

  // now data should be stable. Get set and enter the loop
  sbSpeed   = gps.speed;                //prior read speed
  sbHeading = gps.heading;
  myHeading = sbHeading;
  broadcastLocation(gps, myComment);
  timeOfAPRS = millis();
  digitalWrite(TFT_ONOFF_PIN, tftOnOff); /// this is the TFT display controller set to 255 on boot
  tft.fillScreen(ILI9341_BLACK);
  display();
}

////////////////////////////////////////////////////////////////////// loop()
void loop()
{
  // capture the button press
  if (pushbutton.update()) {
    if (pushbutton.fallingEdge()) {
      digitalWrite(TFT_ONOFF_PIN, 1); //Turn the screen on if it was off
      menu(addresses);
    }
  }

  // get GPS data
  if (gps.sentenceAvailable()) gps.parseSentence();
  if (gps.newValuesSinceDataRead()) {         // go here as the seconds tick
    displayCountDown();
    gps.dataRead();
    gotGPS = true; // prior validate idea... on the to do list
    if (sbEnable) {     
      mySpeed   = round(gps.speed * knotToMPH); // convert knots to MPH and store.
      myHeading = round(gps.heading);           // store the heading
      // SmartBeacon... my way
      if ((sbSpeed != mySpeed)) {               // was speed changed?????
        //go if something changed
        if ((mySpeed > sbSlowSpeed) && (mySpeed < sbFastSpeed)) {
          dTime = (( ( float(sbFastSpeed) / float(mySpeed) ) * (sbFastRate)) * 1000); // - (millis() - timeOfAPRS);
        }
        if (mySpeed >= sbFastSpeed) {         // are we above the fast level
          dTime = (sbFastRate * 1000);        // - (millis() - timeOfAPRS);
        }
      }
      if (sbHeading != myHeading) {             // was heading changed?????????
        uint16_t sbTurnThreshold = sbMinTurnAngle + (sbTurnSlope / mySpeed); // smartbeacon formula
        //  do we send beacon for a direction change??
        int dif = sbHeading - myHeading;
        dif = abs(dif);
        if (abs(dif) > 180) {
          dif = dif - 360;
        }
        dif = abs(dif);
        if (dif > sbTurnThreshold) {                              //did we trun enough?
          if (((millis() - timeOfAPRS) / 1000) > sbMinTurnTime) { //did enough time pass?
            timeOfAPRS = millis() - dTime;        // set the tme so we xmit
            sbHeading = myHeading;
          }
          else {
            Serial.println("not enuf time... reset");
            sbHeading = myHeading;              //not enough time so reset the heading and move on
          }
        }
      }
    }
    display();
    //Serial.printf("Sec: %d Heading %d Knots: %f\n\r", gps.seconds, gps.heading, gps.speed);
    //Serial.printf("Location: %f, %f Heading %d Knots: %f\n\r", gps.latitude, gps.longitude,gps.heading, gps.speed);
    //Serial.printf("dTime %i timeOf %i dT+timeOf %i millis %i\n\r", dTime, timeOfAPRS, dTime + timeOfAPRS, millis());
    //Serial.print((millis() - timeOfAPRS) / 1000);
  }
  // do we xmit?
  if (gotGPS && timeOfAPRS + dTime < millis()) {
    broadcastLocation(gps, myComment );
    timeOfAPRS = millis(); // reset the timer
    //Serial.printf("dTime: %d gps.speed: %f sbSpeed %f sbSlowrate: %d\n\r", dTime, gps.speed, sbSpeed , sbSlowRate);
    // SmartBeacon speed < low speed test and adjust as needed.
    if (sbEnable) {
      sbHeading = gps.heading; // reset the heading milestone.
      sbSpeed = gps.speed * knotToMPH; //store the new speed.
      if (gps.speed < sbSlowSpeed && sbSpeed < sbSlowSpeed && dTime < (sbSlowRate * 1000) ) {
        dTime = 2 * dTime;
        if (dTime > (sbSlowRate * 1000) ) dTime = sbSlowRate * 1000;
      }
    }
    display();
  }
} // Loop end YES the main loop ends here...


/////////////////////////////////////////////////////////////////////////////
//                                                                         //
//       FUNCTIONS from here down                                          //
//                                                                         //
/////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////// squelcher()
// returns the integer value on the SQUEL_PIN
int squelcher() {
  // read the audio level 10 times at 10 millisecond inetervals (100 msec )
  // return the total
  analogReference(SQUEL_REF );
  analogReadResolution(SQUEL_RES);
  analogReadAveraging(SQUEL_AVE);
  int t = 0;
  for (int i = 0; i < 10; i++) {
    t = t + analogRead(SQUEL_PIN);
    delay (10);
  }
  return t;
}


/////////////////////////////////////////////////////////////////////// menuHeader()
// draws the menus
//
void menuHeader(String title, int mStart, int mEnd, int mPick, String menu1[][2]  ) {
  tft.fillScreen(ILI9341_BLUE);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
  tft.setCursor(0, 0);
  tft.println(title);
  for (int i = mStart; i <= mEnd; i++) {
    tft.setCursor(0, (i + 1) * 25);
    if (i == mPick) {
      tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
    }
    else
    {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    //tft.print(String(i) + " ");
    tft.print(menu1[i][0] + menu1[i][1]);
  }
}


/////////////////////////////////////////////////////////////////////// menu()
void menu(const PathAddress *  paths) {
  {
    // lets set it up
    int mStart = 0;   //First menu item
    int mEnd   = 5;   // last menu item
    int mPick  = 0;   // menu choice
    int mB4    = 0;   // line # befor move
    String onOff;
    if (tftOnOff) {
      onOff = "On";
    }
    else {
      onOff = "Off";
    }
    //String displayOnOff = "Display:" + onOff;
    //this defines the menu
    String menu1[][2] = {
      {"Return", ""},
      {"Send/Return", ""},
      {"Packet", ""},
      {"AX.25", ""},
      {"SmartBeacon", ""},
      {"Display: " , onOff}
    } ;
    //now draw it
    menuHeader("Main  Menu", mStart, mEnd, mPick, menu1  );
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
        Serial.printf("mPick: %i, mB4: %i\n\r", mPick, mB4);
        tft.setCursor(0, (mB4 + 1) * 25);
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
        tft.print(menu1[mB4][0] + menu1[mB4][1]);
        tft.setCursor(0, (mPick + 1) * 25);
        tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
        tft.print(menu1[mPick][0] + menu1[mPick][1]);
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
        mB4 = mPick;
      }
      if (pushbutton.update()) {                // button pushed
        if (pushbutton.fallingEdge()) {
          switch (mPick) {                      //handle the button push
            case 0:  // return
              tft.fillScreen(ILI9341_BLACK);
              display();
              digitalWrite(TFT_ONOFF_PIN, tftOnOff); // reset the display
              return;
            case 1:  // send and return
              gps.dataRead();
              broadcastLocation(gps, myComment );
              tft.fillScreen(ILI9341_BLACK);
              timeOfAPRS = millis();
              display();
              digitalWrite(TFT_ONOFF_PIN, tftOnOff); // reset the display
              return;
            case 2: // sendMenu
              packetMenu(paths);
              menuHeader("Main  Menu", mStart, mEnd, 2, menu1  );
              break;
            case 3: // sendMenu
              ax25Menu();
              menuHeader("Main  Menu", mStart, mEnd, 3, menu1  );
              break;
            case 4: // SmartBeacon Menu
              sbMenu();
              menuHeader("Main  Menu", mStart, mEnd, 4, menu1  );
              break;
            case 5: // Display On or Off
              mNumChoice(TFT_ONOFF, &tftOnOff , 0, 1, tftOnOff , String("Display On / Off"), "1 = On              0 = Off");
              if (tftOnOff) {
                onOff = "On";
              }
              else {
                onOff = "Off";
              }
              menu1[5][1] = onOff;
              menuHeader("Main Menu", mStart, mEnd, 5, menu1  );
              break;
          } // switch end
        }   // if (pushbutton.update())
      }     // if (pushbutton.fallingEdge())
    }       // while (true) end
  }         // end of menu function
}

/////////////////////////////////////////////////////////// AX25Menu()
void ax25Menu()
{
  // lets set it up
  int mStart = 0;   //First menu item
  int mEnd   = 6 ;  // last menu item
  int mPick  = 0;   // menu choice
  int mB4    = 0;   // line # before move
  String menu1[][2] = {
    {"Return", ""},
    {"Xmit Dly:", String(axDelay)},
    {"# Flags :", String(axFlags)},
    {"VOX on  :", String(axVoxOn)},
    {"VOX off :", String(axVoxSilent)},
    {"PTT Pin :", String(pttPin)},
    {"Squelch :", String(squelch)}
  };
  //now draw it
  menuHeader("AX.25 Menu", mStart, mEnd, mPick, menu1  );
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
      Serial.printf("mPick: %i, mB4: %i\n\r", mPick, mB4);
      tft.setCursor(0, (mB4 + 1) * 25);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      //tft.print(String(mB4) + " ");
      tft.print(menu1[mB4][0] + menu1[mB4][1]);
      tft.setCursor(0, (mPick + 1) * 25);
      tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
      //tft.print(String(mPick) + " ");
      tft.print(menu1[mPick][0] + menu1[mPick][1]);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      mB4 = mPick;
    }
    if (pushbutton.update()) {                // button pushed
      if (pushbutton.fallingEdge()) {
        switch (mPick) {                      //handle the button push
          // this will use the uint16_t options so arg 2 is a dummy and arg 8 just need a non-zero value.
          case 0:  // return
            tft.fillScreen(ILI9341_BLUE);
            return;
          case 1: // xmit delay
            mNumChoice(AXDELAY, &TimeZone , 10, 1000, axDelay , String("Xmit Delay"), "Xmit Delay in       milliseconds", axDelay, &axDelay, true);
            menu1[1][1] = String(axDelay);
            menuHeader("AX.25 Menu", mStart, mEnd, 1, menu1  );
            break;
          case 2: // flags
            mNumChoice(AXFLAGS, &TimeZone , 1, 100, axFlags , String("Flags"), "# of AX'25 frame    start flags", axFlags, &axFlags, true);
            menu1[2][1] = String(axFlags);
            menuHeader("AX.25 Menu", mStart, mEnd, 2, menu1  );
            break;
          case 3:
            mNumChoice(AXVOXON, &TimeZone , 0, 1000, axVoxOn , String("VOX On"), "VOX On milliseconds. >0 for VOX.        PTT Pin must be set to o to turn on VOX)", axVoxOn, &axVoxOn, true);
            menu1[3][1] = String(axVoxOn);
            menuHeader("AX.25 Menu", mStart, mEnd, 3, menu1  );
            break;
          case 4:
            mNumChoice(AXVOXSILENT, &TimeZone , 0, 1000, axVoxSilent , String("VOX Silent"), "VOX Silent milliseconds", axVoxSilent, &axVoxSilent, true);
            menu1[4][1] = String(axVoxSilent);
            menuHeader("AX.25 Menu", mStart, mEnd, 4, menu1  );
            break;
          case 5:
            mNumChoice(PTT_PIN , &TimeZone , 0, 23, pttPin , String("PTT Pin #"), "PTT Pin (usually 13)0 (zero) and        VOX On >0 for VOX", pttPin, &pttPin, true);
            menu1[5][1] = String(pttPin);
            menuHeader("AX.25 Menu", mStart, mEnd, 5, menu1  );
            break;
          case 6:
            mNumChoice(SQUELCH , &TimeZone , 0, 1000, squelch , String("Squelch Threshold"), "Squelch (0 - 1000)  0 (zero) for off", squelch, &squelch, true);
            menu1[6][1] = String(squelch);
            menuHeader("AX.25 Menu", mStart, mEnd, 6, menu1  );
            break;
        } // switch end
      }   // if (pushbutton.update())
    }     // if (pushbutton.fallingEdge())
  }       // while (true) end
}         // end of menu function



/////////////////////////////////////////////////////////// sbMenu()
void sbMenu()
{
  // lets set it up
  int mStart = 0;   //First menu item
  int mEnd   = 8 ;  // last menu item
  int mPick  = 0;   // menu choice
  int mB4    = 0;   // line # before move
  String enabled;
  if (sbEnable) {
    enabled     = "Enabled";
  }
  else {
    enabled     = "Disabled";
  }
  String menu1[][2] = {
    {"Return", ""},
    {enabled, ""},
    {"fSpd :", String(sbFastSpeed)},
    {"fRate:", String(sbFastRate)},
    {"sSpd :", String(sbSlowSpeed)},
    {"sRate:", String(sbSlowRate)},
    {"tTime:", String(sbMinTurnTime)},
    {"tAgle:", String(sbMinTurnAngle)},
    {"Slope:", String(sbTurnSlope)}
  };

  //now draw it
  menuHeader("SB Menu", mStart, mEnd, mPick, menu1  );
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
      Serial.printf("mPick: %i, mB4: %i\n\r", mPick, mB4);
      tft.setCursor(0, (mB4 + 1) * 25);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      tft.print(menu1[mB4][0] + menu1[mB4][1]);
      tft.setCursor(0, (mPick + 1) * 25);
      tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
      tft.print(menu1[mPick][0] + menu1[mPick][1]);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      mB4 = mPick;
    }
    if (pushbutton.update()) {                // button pushed
      if (pushbutton.fallingEdge()) {
        switch (mPick) {                      //handle the button push
          // this will use the uint16_t options so arg 2 is a dummy and arg 8 just need a non-zero value.
          case 0:
            tft.fillScreen(ILI9341_BLUE);
            return;
          case 1: // Enabled
            mNumChoice(SB_ENABLE, &sbEnable , 0, 1, sbEnable , String("SB Enabled"), "1 = enabled         0 = disabled");
            if (!sbEnable) {
              sTime = EEPROM.readInt(XMIT_TIME); // seconds
              dTime = sTime * 1000; // store the start time into dTime (delay) in milliseconds
              timeOfAPRS = millis();
              menu1[1][0]    = "Disabled";
            }
            else {
              menu1[1][0]     = "Ensabled";
            }
            menuHeader("SB Menu", mStart, mEnd, 1, menu1  );
            break;
          case 2: // FastSpeed
            mNumChoice(SBFAST_SPEED, &TimeZone , 10, 100, sbFastSpeed , String("Fast Speed"), "MPH. At or above this speed xmit delay = Fast Rate", sbFastSpeed, &sbFastSpeed, true);
            menu1[2][1] = String(sbFastSpeed);
            menuHeader("SB Menu", mStart, mEnd, 2, menu1  );
            break;
          case 3:
            mNumChoice(SBFAST_RATE, &TimeZone , 10, 600, sbFastRate , String("Fast Rate"), "Seconds. Xmit delay at or above Fast Speed", sbFastRate, &sbFastRate, true);
            menu1[3][1] = String(sbFastRate);
            menuHeader("SB Menu", mStart, mEnd, 3, menu1  );
            break;
          case 4:
            mNumChoice(SBSLOW_SPEED, &TimeZone , 1, 20, sbSlowSpeed , String("Slow Speed"), "MPH. At or below this speed xmit delay = Slow Rate", sbSlowSpeed, &sbSlowSpeed, true);
            menu1[4][1] = String(sbSlowSpeed);
            menuHeader("SB Menu", mStart, mEnd, 4, menu1  );
            break;
          case 5:
            mNumChoice(SBSLOW_RATE, &TimeZone , 1000, 5000, sbSlowRate , String("Slow Rate"), "Seconds. Xmit delay at or below Slow Speed", sbSlowRate, &sbSlowRate, true);
            menu1[5][1] = String(sbSlowRate);
            menuHeader("SB Menu", mStart, mEnd, 5, menu1  );
            break;
          case 6:
            mNumChoice(SBTURN_TIME, &TimeZone , 5, 30, sbMinTurnTime , String("Min. Turn Time"), "Seconds. Minimum time between ximts for a turn", sbMinTurnTime, &sbMinTurnTime, true);
            menu1[6][1] = String(sbMinTurnTime);
            menuHeader("SB Menu", mStart, mEnd, 6, menu1  );
            break;
          case 7:
            mNumChoice(SBTURN_ANGLE, &TimeZone , 5, 30, sbMinTurnAngle , String("Min. Turn Angle"), "Degrees. Minimu degrees required bedofr turn xmit", sbMinTurnAngle, &sbMinTurnAngle, true);
            menu1[7][1] = String(sbMinTurnAngle);
            menuHeader("SB Menu", mStart, mEnd, 7, menu1  );
            break;
          case 8: // SmartBeacon Menu
            mNumChoice(SBTURN_SLOPE, &TimeZone , 200, 300, sbTurnSlope , String("Turn Slope"), "Turn slope. See SmartBeacon documentation", sbTurnSlope, &sbTurnSlope, true);
            menu1[8][1] = String(sbTurnSlope);
            menuHeader("SB Menu", mStart, mEnd, 8, menu1  );
            break; //return;
        } // switch end
      }   // if (pushbutton.update())
    }     // if (pushbutton.fallingEdge())
  }       // while (true) end
}       // end of menu function

///////////////////////////////////////////////////////////////////// packetMenu()

void packetMenu(const PathAddress *  paths)
{
  // lets set it up
  int mStart = 0;   //First menu item
  int mEnd   = 8;   // last menu item
  int mPick  = 0;   // menu choice
  int mB4    = 0;   // line # befor move
  String symName;   // symbol name holder
  int symNum = 0;   // curnt symbol # in array
  //this defines the menu
  for (int i = 0; i < NUM_SYMBOLS; i++) { // find the symbol, store the name into symName
    if ((symbol == symbols[i][2][0]) && (symTable == symbols[i][1][0]) ) {
      symName   = String(symbols[i][0]);
      symNum = i;
      break;
    }
  }
  String menu1[][2] = {
    {"Return", ""},
    {"UTC  :", String(TimeZone)},
    {"Delay:" , String(sTime)},
    {paths[1].callsign, ""},
    {"SSID :" , String(paths[1].ssid)},
    {paths[0].callsign, ""},
    {"SSID :", String(paths[0].ssid)},
    {symName, ""},
    {"Comment", "" }
  };
  //now draw it
  menuHeader("Packet Menu", mStart, mEnd, mPick, menu1  );
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
      Serial.printf("mPick: %i, mB4: %i\n\r", mPick, mB4);
      tft.setCursor(0, (mB4 + 1) * 25);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      tft.print(menu1[mB4][0] + menu1[mB4][1]);
      tft.setCursor(0, (mPick + 1) * 25);
      tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
      tft.print(menu1[mPick][0] + menu1[mPick][1]);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      mB4 = mPick;
    }
    if (pushbutton.update()) {                // button pushed
      if (pushbutton.fallingEdge()) {
        char callAlpha[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890 ";  // Call Alpha array to make the call
        char commAlpha[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz1234567890 *";  // Meaasage Alpha array to make the call
        switch (mPick) {                      //handle the button push
          case 0:  // return
            tft.fillScreen(ILI9341_BLUE);
            return;
          case 1: // utc offset set
            mNumChoice(UTC_OFFSET, &TimeZone, -12, 14, TimeZone, "UTC Offset", "Hours. Set to actual offset including DST");
            menu1[1][1] = String(TimeZone);
            menuHeader("Packet Menu", mStart, mEnd, 1, menu1  );
            break;
          case 2: // xmit delay
            mNumChoice(XMIT_TIME, &TimeZone , 10, 600, sTime , String("Xmit Delay"), "Seconds between transissions (10-600) if SmartBeacon disabled", sTime, &sTime, true );
            dTime = sTime * 1000;
            menu1[2][1] = String(sTime);
            menuHeader("Packet Menu", mStart, mEnd, 2, menu1  );
            break;
          case 3:
            mCommChoice(MY_CALL, "My Call", sCall, callAlpha, strlen(callAlpha), 7, 0) ;
            menu1[3][0] = paths[1].callsign;
            menuHeader("Packet Menu", mStart, mEnd, 3, menu1  );
            break;
          case 4:
            mNumChoice(MY_SSID, &addresses[1].ssid, 0, 15, addresses[1].ssid , String(paths[1].callsign) + "-SSID", "SSID for:" + String(paths[1].callsign) );
            menu1[4][1] = String(addresses[1].ssid);
            menuHeader("Packet Menu", mStart, mEnd, 4, menu1  );
            break;
          case 5:
            mCommChoice(DEST_CALL, "Dest. Call", dCall, callAlpha, strlen(callAlpha), 7, 0) ;
            menu1[5][0] = paths[0].callsign;
            menuHeader("Packet Menu", mStart, mEnd, 5, menu1  );
            break;
          case 6:
            mNumChoice(DEST_SSID, &addresses[0].ssid, 0, 15, addresses[0].ssid , String(paths[0].callsign) + "-SSID", "SSID for:" + String(paths[0].callsign) );
            menu1[6][1] = String(addresses[0].ssid);
            menuHeader("Packet Menu", mStart, mEnd, 6, menu1  );
            break;
          case 7:
            mSymChoice("Symbol", symName, symNum);
            menu1[7][0] = symName;
            menuHeader("Packet Menu", mStart, mEnd, 7, menu1  );
            break;
          case 8: // this will do the message
            mCommChoice(COMMENT, "Comment:", myComment, commAlpha, strlen(commAlpha), 35, 3) ;
            menuHeader("Packet Menu", mStart, mEnd, 8, menu1  );
            break;
        } // switch end
      }   // if (pushbutton.update())
    }     // if (pushbutton.fallingEdge())
  }       // while (true) end
}          // end of menu function

////////////////////////////////////////////////////////////mSymChoice
void mSymChoice(String title, String sNameNow, int sNumNow  ) {
  /*
   * Menu choice for the symbols....
   *
   */
  // this is the setup()

  tft.fillScreen(ILI9341_BLUE);     // now draw the screen
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
  tft.setCursor(0, 0);
  tft.println(title);
  tft.setCursor(0, 25);
  tft.print("Now: ");
  tft.setCursor(80, 25);
  tft.print(sNameNow);
  tft.setCursor(0, 75);
  tft.print("New: ");
  tft.setCursor(80, 75);
  tft.setTextColor( ILI9341_BLUE, ILI9341_WHITE);
  tft.print(symbols[sNumNow][0]);
  int sNumNew = sNumNow;
  while (true) {
    if (pushbutton.update()) {                // button pushed
      if (pushbutton.fallingEdge()) {
        if (sNumNew != sNumNow) {      // we have a new symbol to save
          Serial.print(symbols[sNumNew][1]);
          Serial.print(symbols[sNumNew][2]);
          Serial.println(symbols[sNumNew][0]);
          EEPROM.update(SYM_TABLE, symbols[sNumNew][1][0]);
          EEPROM.update(SYMBOL, symbols[sNumNew][2][0]);
          symTable = symbols[sNumNew][1][0];
          symbol = symbols[sNumNew][2][0];
          Serial.println(symTable);
          Serial.print(symbol);
        }
        tft.fillScreen(ILI9341_BLUE);
        // display();
        return;

      }
    }
    unsigned char result = rotary_process(); // rotated?
    if (result) {
      if (result == DIR_CCW) {
        --sNumNew;
        if (sNumNew < 0 ) sNumNew = NUM_SYMBOLS - 1;
      }
      else {
        ++sNumNew;
        if (sNumNew >= NUM_SYMBOLS ) sNumNew = 0;
      }
      tft.setCursor(80, 75 );
      tft.setTextColor( ILI9341_WHITE, ILI9341_BLUE);
      tft.print( "           " );
      tft.setCursor(80, 75 );
      tft.setTextColor( ILI9341_BLUE, ILI9341_WHITE);
      tft.print(symbols[sNumNew][0]);
    }
  }
}


////////////////////////////////////////////////////////////mNumChoice
void mNumChoice(int eePromAddress
                , int8_t *variable    // pointer to the int8_t variable to be edited
                , int lTZ             // lower choice limit
                , int uTZ             // upper choice limit
                , int nTZ             // this is the current value that is dispalyed
                , String title        // size 3 at top of screen
                , String help         // size 2 below input area
                , uint16_t TTT        // optional current value
                , uint16_t *variable2 // optional pointer to uint32_t value to be edited
                , bool e16Bit )       // optional default = false true for uint16_t16 bit
{
  /*
   * Menu choice for numeric entries entries
   * Pararmaters described above.
   * Optional parameters are all or none.
   * Optional 'flag' is e16Bit = true
   */
  tft.fillScreen(ILI9341_BLUE);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
  tft.setCursor(0, 0);
  tft.println(title);
  tft.setCursor(0, 25);
  tft.print("Now: ");
  if (e16Bit) {
    tft.print(*variable2);
  }
  else {
    tft.print(*variable);
  }
  tft.setCursor(0, 75);
  tft.print("New: ");
  tft.setCursor(0, 125);
  tft.setTextSize(2);
  tft.println("Rotate to New Value");
  tft.println("Push to accept");
  tft.println("");
  tft.println(help);
  tft.setTextSize(3);
  tft.setCursor(100, 75);
  tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
  tft.print(nTZ);
  while (true) {
    if (pushbutton.update()) {                // button pushed
      if (pushbutton.fallingEdge()) {
        if (e16Bit) {                      // this is the int16_t calcs.
          if (*variable2 != nTZ) {           // *variable2 points to dTime
            *variable2 = nTZ;
            EEPROM.updateInt(eePromAddress, *variable2);
            *variable2 = *variable2;
          }
        }
        else {                                // this is the int8_t calcs
          if (*variable != nTZ) {
            *variable = nTZ;
            EEPROM.update(eePromAddress, *variable);
          }
        }
        tft.fillScreen(ILI9341_BLUE);
        //display();
        return;
      }
    }
    unsigned char result = rotary_process(); // rotated?
    if (result) {
      if (result == DIR_CCW) {
        if (--nTZ < lTZ) {
          nTZ = uTZ;
        }
      }
      else {
        if (++nTZ > uTZ) {
          nTZ = lTZ;
        }
      }
      //tft.setTextColor(ILI9341_WHITE,ILI9341_BLUE);
      //tft.setCursor(0, 75);
      //tft.print("New: ");
      tft.setCursor(100, 75);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      tft.print("   ");
      tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
      tft.setCursor(100, 75);
      tft.print(nTZ);

    }
   
  }
  delay(1000);
  tft.fillScreen(ILI9341_BLUE);
  //display();
  return;
}

//////////////////////////////////////////////////////////// displayCountDown()
void displayCountDown()
{
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(170, 0);
  uint32_t tLeft = (dTime - (millis() - timeOfAPRS)) / 1000;
  int sec = tLeft % 60;
  int min = tLeft / 60;
  char buf[6];
  sprintf( buf, "%2d:%2d", min, sec);
  tft.print(buf);
  tft.setCursor(170, line * 1);
  tLeft = dTime / 1000;
  sec = tLeft % 60;
  min = tLeft / 60;
  sprintf( buf, "%2d:%2d", min, sec);
  tft.print(buf);
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
  tft.setCursor(60, line * 7);
  tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
  tft.setTextSize(4);
  char buf[6];
  //sprintf( buf, "%2d MPH", int(mySpeed ));// for testing
  sprintf( buf, "%2d MPH", int(round(gps.speed * knotToMPH) ));
  tft.print(buf);
  tft.setCursor(40, line * 9);
  tft.setTextSize(4);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  sprintf( buf, "%3d deg", int(round(gps.heading) ));
  tft.print(buf);
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


/////////////////////////////////////////////////////////////////////// broadcastLocation()
//Function to broadcast your location
void broadcastLocation(GPS &gps, const char *bcomment)
{
  // If above 5000 feet switch to a single hop path
  int nAddresses;
  // load the user data variables into the addresses[] array for xmit
  //
  addresses[0].callsign = dCall;
  addresses[0].ssid = EEPROM.read(DEST_SSID);
  addresses[1].callsign = sCall;
  addresses[1].ssid = EEPROM.read(MY_SSID);
  symbol =  EEPROM.read(SYMBOL);
  symTable =  EEPROM.read(SYM_TABLE);
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
  int squ = squelcher();
  uint8_t squCount = 0;
  while (squelch != 0 && squ > squelch && squCount++ < 50){
      //check the audio / squelch if it is busy. 
      //sqCount is a 5 second safety
      squ = squelcher();
      Serial.print(squCount);
      Serial.print(" : ");
      Serial.println(squ);
      
  }
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

///////////////////////////////////////////////////////////////////// mCommChoice()
void mCommChoice(int eePromAddress, String title, char *vnow, char * alpha, int alength, int vNowLength, int rotation) {
  /*
   * Menu choice for the alpha numeric fields sucha as the call signs and comments....
   *
   */
  // this is the setup()
  uint16_t white    = ILI9341_WHITE;
  uint16_t yellow   = ILI9341_YELLOW;
  char cNew[vNowLength];                // holds new comment
  strcpy(cNew, vnow);           // put old into new
  String mCallExit[] = {"Continue", "Exit", "Exit/Save"};  // exit choices
  int nLetter = 0;              // pointer to current letter in new call
  int letter = 0;               // ptr for the alpha[] array when rotating.
  int mCallExitChoice = 0;      // exit choice pointer
  tft.setRotation(rotation);
  tft.fillScreen(ILI9341_BLUE); // now draw the screen
  tft.setTextSize(3);

  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_NAVY);
  tft.println(title);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
  tft.setCursor(0, 25);
  tft.print("Now: ");
  tft.setCursor(0, 50);
  tft.print(vnow);
  tft.setCursor(0, 100);
  tft.print("New: ");
  tft.setCursor(0, 125);
  for (int i = 0; i < vNowLength; i++) {  // this is a predraw before we enter the loop
    if (i == nLetter) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      tft.print(cNew[i]);
      tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
    }
    else {
      tft.print(cNew[i]);
    }
  }
  // this is the loop() for the comment editor
  while (true) {
    unsigned char result = rotary_process(); ////////rotated
    if (result) {
      if (result == DIR_CCW) {
        --nLetter;
      }
      else {
        ++nLetter;
      }
      if (nLetter < 0 || nLetter == vNowLength) { // display exit choice
        tft.setCursor(0, 125);
        for (int i = 0; i < vNowLength; i++) {
          tft.print(cNew[i]);
        }
        tft.setCursor(20, 200);
        nLetter++;
        tft.print( "         " );
        tft.setCursor(20, 200);
        tft.print( mCallExit[mCallExitChoice]);
        while (true) {
          unsigned char result = rotary_process(); // rotated?
          if (result) {
            if (++mCallExitChoice > 2) mCallExitChoice = 0;
            tft.setCursor(20, 200); // display exit options
            tft.setTextColor( ILI9341_WHITE, ILI9341_BLUE);
            tft.print( "         " );
            tft.setTextColor( ILI9341_BLUE, ILI9341_WHITE);
            tft.setCursor(20, 200);
            tft.print( mCallExit[mCallExitChoice]);
          }
          if (pushbutton.update()) {                // button pushed
            if (pushbutton.fallingEdge()) {
              switch (mCallExitChoice) {
                case 0: //continue
                  tft.setCursor(20, 200);
                  tft.setTextColor( ILI9341_WHITE, ILI9341_BLUE); // blank the exit choice
                  tft.print( "         " );
                  break; //breaks the switch
                case 1: // exit
                  tft.setRotation(0);
                  tft.fillScreen(ILI9341_BLACK);
                  display();
                  return;
                case 2: //exit and save
                  strcpy(vnow, cNew);
                  for (int i = 0; i < vNowLength; i++) {
                    EEPROM.update(eePromAddress + i, char(vnow[i]));
                  }
                  tft.setRotation(0);
                  tft.fillScreen(ILI9341_BLACK);
                  display();
                  return;
              } // switch end
              break;  // breaks the exit while true loop
            } // falling edge if
          } // pb update if
        } //while true

        if (nLetter < 0) nLetter = vNowLength - 1;
        if (nLetter > vNowLength) nLetter = 0;
        mDispComment(nLetter, cNew, white, vNowLength);
      }
      else {
        mDispComment(nLetter, cNew, white, vNowLength);
      }     // else
    }       // if result end
    if (pushbutton.update()) { //////////////////////// button pushed
      if (pushbutton.fallingEdge()) {
        for (int i = 0; i < alength; i++) { // find the letter under pb in alpha
          if (cNew[nLetter] == alpha[i]) {
            letter = i; // found it, set the letter to is and get out of the for loop
          }
        }
        mDispComment(nLetter, cNew, yellow, vNowLength);
        while (true) {
          unsigned char result = rotary_process(); // rotated?
          if (result) {
            if (result == DIR_CCW) {
              --letter;
              if (letter < 0) letter = alength;
            }
            else {
              ++letter;
              if (letter > alength) letter = 0;
            }
            cNew[nLetter] = alpha[letter];        // put the new letter into the cNew string
            mDispComment(nLetter, cNew, yellow, vNowLength); // display it
          }  // if rotated result

          if (pushbutton.update()) {     // button pushed
            if (pushbutton.fallingEdge()) {
              break;
            }
          }  // pushbutton
        }    // while true letter edit loop
      }      // falling edge if
    }        // pushbutton if update
  }          // While true loop
}            // mMessChoice()

void mDispComment(int nLetter, char *cNew, uint16_t color, int vNowLength) {
  tft.setCursor(0, 125);
  tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
  for (int i = 0; i < vNowLength; i++) {
    if (i == nLetter) {
      tft.setTextColor(color, ILI9341_BLUE);
      tft.print(cNew[i]);
      tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
    }
    else {
      tft.print(cNew[i]);
    }
  }  // for loop disp comment
}  // function end

