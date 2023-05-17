// **********************************************************
// ******************    Flysky Rx Code   *******************
//               by midelic on RCgroups.com
//   Thanks to PhracturedBlue,ThierryRC,Dave1993,and the team
//    of OpenLRS project and  Hasi for PPM encoder
//                 Modified by Jacek Kosek
// **********************************************************
#include <EEPROM.h>
#define SerialBaudRate 115200 //115.200 baud serial port speed

//WLToys transmitter
//Channel 1 (Right X) AIR - airlons it theory (for my Cessna it's RUD)
//Channel 2 (Right Y) ELE - elevation (horisontal stabiliser)
//Channel 3 (Left  Y) THR - throttle
//Channel 4 (Left  X) RUD - ruther but when right button pressed it
//                switches RUD signal values:
//          short long values
//            0     0     1000  -   2000
//            0     1   -31766  - -30770
//            1     1   -15382  - -14386
//            1     0    17386  -  18382
//Channel 5 aroung 600 (transmmiter voltage or signal?)
//Channel 6 alternating two values above 1000 and around 40
//Channel 7 always 0 (Good test for transmmiter)
//Cannnel 8 is 0 except when channel 4 reach maximum then is 100

const bool cDbgHw = true;
const bool cDbgIn = true;
const bool cDbgOut = true;

//pins configuration
//radio
static const int GIO_pin   = 13;
static const int SDI_pin   = 19;
static const int SCLK_pin  = 18;
static const int CS_pin    = 17;
//IO
static const int bind_pin  = 2;
static const int RLED_pin  = 21;
static const int MOT1_pin  = 12;
static const int MOT2_pin  = 11;
static const int MOT3a_pin = 9;
static const int MOT3b_pin = 10;

//timers has to be divided by 16
const uint32_t failsafeTimeout = 200;     // after this time failsafe mode is set
const uint32_t channelChangeTimeout = 30; // after this time next chanel is listen

//shortcut
inline void rLedOff() { digitalWrite(RLED_pin, HIGH);};
inline void rLedOn()  { digitalWrite(RLED_pin, LOW) ;};


//############## GLOBALS ################

//To transfer information from main loop into Interupt routine 
int8_t INTstart=1; //starting index of for array
struct INTdata {
    uint8_t timer;
    int8_t next;
} static INTval[4] = {{0,-1},{0,-1},{0,-1},{0,-1}};

class FlySkyRx {
  public:
    void init();
    bool received();
    void listenNext();
    void bind();
    int data[8] = {1500,1500,1000,1500,1500,1500,1500,1500};      
    uint32_t id;
  private:
    uint32_t idReceived;
    uint8_t readPacket();
} rf;

//############### SETUP ################
void setup() {
    noInterrupts();

    if(cDbgHw | cDbgIn | cDbgOut)
      Serial.begin(SerialBaudRate);//for debug

    pinMode(RLED_pin, OUTPUT);
    pinMode(MOT1_pin, OUTPUT);
    pinMode(MOT2_pin, OUTPUT);
    pinMode(MOT3a_pin,OUTPUT);
    pinMode(MOT3b_pin,OUTPUT);

    //-----------------TIMER2 config
    TCCR2A = 0x00; //Configure in normal mode (pure increment counting, no PWM etc.)
    TIMSK2 = 0x01; //Disable Compare Match A interrupt enable (only want overflow)
    TCCR2B = 0x04; //Prescaler clock divided by: 1=>1 32kHz, 2=>8 4kHz, 3=>32 1kHz,
                   //      4=>64 500Hz, 5=>128 250Hz, 6=>256 125Hz, 7=>1024 32.5Hz
    ASSR   = 0x00; //Select clock source: internal I/O clock
    TCNT2  = 0;    //Max time for first count

    interrupts();
 
    rf.init();
    
    //if(testBindJumper()) {
    //    rf.bind();    
    //}
    //rf.id = fromEEPROM(0);
    rf.id = 0x942435e2; //WLTOYS transmmiter;
    
    if(cDbgHw) Serial.println(rf.id,HEX);
    rf.listenNext();
}

//############ MAIN LOOP ##############
void loop() {
    static uint32_t failsafeTime;

    if (rf.received()) {
      if (cDbgIn) {
        for (uint8_t i=0; i<8; i++) {
            Serial.print(rf.data[i]);
            Serial.print("\t");
        }
      }
      setActuators();
      if(cDbgIn | cDbgOut) Serial.println("");
      failsafeTime = millis();
    } else {
      if( (millis()-failsafeTime) > failsafeTimeout ){
        //Failsave mode
        for (uint8_t i=0; i<8; i++) {
          rf.data[i]=1500;
        }
        rf.data[2]=1000;

        if(cDbgIn) Serial.println("Failsave!");
        failsafeTime = millis();
        setActuators();
        if(cDbgIn | cDbgOut) Serial.println("");
      }
    }
    
}


void setActuators() {
  int16_t motor1;
  int16_t motor2;
  int16_t motor3;
  uint8_t motDir;
  int32_t diff;

  //two motors

  // calculating difference of the speed for 2 main motors
  diff = rf.data[0] - 1500;
  
  // For DIY transmitter more options
  if( rf.data[6] > 100 ) {

    //using DIY trm
    diff += (rf.data[3] - 1500) / 2;

    //coriolis force from back propeler compensation set by right pot
    //1724 looks OK for potentiometer
    diff += ((int32_t)(rf.data[4] - 1500) * (rf.data[1] - 1500)) / 1024 / 1;
              
  }
  //multiply by throtle and correct output
  diff = ( diff * (int32_t)(rf.data[2] - 1000)) / 1024 / 4;
  
  motor1 = ((rf.data[2]-1000)-diff)/5;
  motor2 = ((rf.data[2]-1000)+diff)/5;
  
  motor1 = constrain(motor1, 0, 255);
  motor2 = 255-constrain(motor2, 0, 255);

  //setting data (motor 1 and 3) for interupt routine
  if(motor1>motor2) {
      noInterrupts();
      INTstart = ( motor2 == 0 ) ? 2 : 0;
      INTval[0].timer = 255-motor2;
      INTval[2].timer = 255-motor1+motor2;
      INTval[3].timer = motor1;
      INTval[0].next = 2;
      INTval[2].next = ( motor1 == 255 ) ? -1 : 3;
      INTval[3].next = -1;
      interrupts();
  }
  if(motor1<motor2) {
      noInterrupts();
      INTstart = ( motor1 == 0 ) ? 1 : 0;
      INTval[0].timer = 255-motor1;
      INTval[1].timer = 255-motor2+motor1;
      INTval[3].timer = motor2;
      INTval[0].next = 1;
      INTval[1].next = ( motor2 == 255 ) ? -1 : 3;
      INTval[3].next = -1;
      interrupts();
  }
  if(motor1==motor2) {
      noInterrupts();
      INTval[0].timer = 255-motor1;
      INTval[3].timer = motor1;
      INTval[0].next = 3;
      INTval[3].next = -1;
      interrupts();
  }

  //motor3
  motDir = (rf.data[1] > 1500) ? LOW : HIGH;
  motor3 = (abs(rf.data[1]-1500))/2;
  motor3 = constrain(motor3, 0, 255);
  motor3 = (motor3 < 8) ? 0 : motor3;
  if (motDir) {
      analogWrite(MOT3a_pin, motor3);
      analogWrite(MOT3b_pin, 0);
  } else {
      analogWrite(MOT3a_pin, 0);
      analogWrite(MOT3b_pin, motor3);        
  }
  if (cDbgOut) {
    Serial.print("\tm1: ");
    Serial.print(motor1);
    Serial.print("\tm2: ");
    Serial.print(motor2);
    Serial.print("\tm3: ");
    Serial.print(motor3);
    Serial.print("\t");
    Serial.print(motDir);
    Serial.print(" d: ");
    Serial.print(diff);
  }
}


//check presence of bind jumper
bool testBindJumper() {
    pinMode(bind_pin, INPUT_PULLUP);//pull up
    if ( digitalRead(bind_pin) == LOW) {
        delay(10);
        return true;
    }
    return false;
}

//TIMER2 interupt rouitine
ISR(TIMER2_OVF_vect) {
    const bool mot1on[4] = {true , false, true, false};
    const bool mot2on[4] = {false, false, true, true };

    static int8_t idx = 1;
    static INTdata conf[4] = {{0,-1},{0,-1},{0,-1},{0,-1}};

    TCNT2 = conf[idx].timer; //Reset Timer

    digitalWrite(MOT1_pin,mot1on[idx]);
    digitalWrite(MOT2_pin,mot2on[idx]);

    if ( conf[idx].next == -1 ) {
        idx = INTstart;
        conf[0] = INTval[0];
        conf[1] = INTval[1];
        conf[2] = INTval[2];
        conf[3] = INTval[3];
    } else {
        idx = conf[idx].next;
    }
}

uint32_t fromEEPROM(unsigned char addr){
    uint32_t ret;
    ret  = (uint32_t)EEPROM.read(addr + 0) <<  0;
    ret += (uint32_t)EEPROM.read(addr + 1) <<  8;
    ret += (uint32_t)EEPROM.read(addr + 2) << 16;
    ret += (uint32_t)EEPROM.read(addr + 3) << 24;
    return ret;
}

