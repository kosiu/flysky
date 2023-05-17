// **********************************************************
// ******************    Flysky Rx Code   *******************
//               by midelic on RCgroups.com
//   Thanks to PhracturedBlue,ThierryRC,Dave1993,and the team
//    of OpenLRS project and  Hasi for PPM encoder
//           Modified by Jacek Kosek 2017-2018
// **********************************************************


#include <EEPROM.h>
#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed

static const bool cDbgHw = true;
static const bool cDbgAdc = true;
static const bool cDbgAna = true;
static const bool cDbgOut = true;
static const bool cDbgCal = true;

//pins configuration
//Radio
static const int GIO_pin   = 12;
static const int SDI_pin   = 9;
static const int SCLK_pin  = 10;
static const int CS_pin    = 11;
//Digital In
static const int tlb_pin  = 7;
static const int tlf_pin  = 8;
static const int flu_pin  = 6;
static const int fru_pin  = 5;
static const int frd_pin  = 21;
//Digital Out
static const int wled_pin = 2;
static const int rled_pin = 3;

static const unsigned int sendPeriod = 25750; // 1460 us * 17?

class Ain {
  public:
    void init();
    void out(); //return calculated analog positions 1000 to 2000
    long signals[8] = {1500,1500,1000,1500,1500,1500,1500,1500};
    unsigned char count[6];
    bool calibration(bool button);
    void memCopy();
    void printChConfig();
    struct InputChannelConfig {
      bool isReversed;
      unsigned char noSegments;
      int outPoint[1]; //assuming that first point is alvays 0 and last 1000
      int  inPoint[3];
               //isReversed seg  out       in
    } chCfg[6] = { { false, 2, {500}, {6, 494, 817} },
                   {  true, 2, {500}, {6, 513, 839} },
                   { false, 1, {  0}, {6, 825,   0} },
                   {  true, 1, {  0}, {6, 896,   0} },
                   { false, 1, {  0}, {6, 823,   0} },
                   {  true, 1, {  0}, {6, 790,   0} }};
} adc;

class FlySkyTx {
  public:
    void init();
    void transmit();
    unsigned char data[8] = {1500,1500,1000,1500,1500,1500,1500,1500};  
    uint32_t id;
  private:
    void writePacket(uint8_t type);
} rf;

//shortcuts
inline void rLedOff() { digitalWrite(rled_pin, HIGH);};
inline void rLedOn()  { digitalWrite(rled_pin, LOW) ;};
inline void wLedOff() { digitalWrite(wled_pin, HIGH);};
inline void wLedOn()  { digitalWrite(wled_pin, LOW) ;};

//############### SETUP ################
void setup() {
    noInterrupts();

    if( cDbgAdc | cDbgAna | cDbgOut | cDbgHw | cDbgCal)
      Serial.begin(SERIAL_BAUD_RATE);//for debug

    pinMode(rled_pin, OUTPUT);
    pinMode(wled_pin, OUTPUT);

    pinMode(tlb_pin, INPUT_PULLUP);
    pinMode(tlf_pin, INPUT_PULLUP);
    pinMode(flu_pin, INPUT_PULLUP);
    pinMode(fru_pin, INPUT_PULLUP);
    pinMode(frd_pin, INPUT_PULLUP);

    for (unsigned char i=0; i<6; i++) {
      adc.chCfg[i].inPoint[0]  = EEPROM.read(i*6+0) << 0;
      adc.chCfg[i].inPoint[0] += EEPROM.read(i*6+1) << 8;
      adc.chCfg[i].inPoint[1]  = EEPROM.read(i*6+2) << 0;
      adc.chCfg[i].inPoint[1] += EEPROM.read(i*6+3) << 8;
      adc.chCfg[i].inPoint[2]  = EEPROM.read(i*6+4) << 0;
      adc.chCfg[i].inPoint[2] += EEPROM.read(i*6+5) << 8;
    }


    rf.init();
    adc.init();

    interrupts();
}

//############ MAIN LOOP ##############
void loop() {
  static unsigned long timer = 0;
  static bool isCalibrating = false;

  bool fluSwitch = !digitalRead(flu_pin);
  if( fluSwitch | isCalibrating ) {isCalibrating = adc.calibration(fluSwitch);}
  else if(micros()-timer>sendPeriod){
    timer = micros();
    rf.transmit();

    //adc.memCopy();
    adc.out();

    if(cDbgAna){
      for (int i=0; i<6; i++) {
          Serial.print(adc.count[i]);
          Serial.print(" ");      
          Serial.print(adc.signals[i]);
          Serial.print(" ");
      }
    }
    digitalWrite(rled_pin,digitalRead(tlf_pin));
    digitalWrite(wled_pin,digitalRead(fru_pin));

    if( cDbgAdc | cDbgAna | cDbgOut ) Serial.println("");
  }
  
}

  
