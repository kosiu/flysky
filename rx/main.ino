// **********************************************************
// ******************    Flysky Rx Code   *******************
//               by midelic on RCgroups.com
//   Thanks to PhracturedBlue,ThierryRC,Dave1993,and the team
//    of OpenLRS project and  Hasi for PPM encoder
//                 Modified by Jacek Kosek
// **********************************************************
#include <optiboot.h>
#define DEBUG
#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed


//this table can be local to function: listenNextChannel() it's here to dont blur the code
//it stores FlySky channels mapping tabele
static const uint8_t tx_channels[16][16] = {
    {0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
    {0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
    {0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
    {0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
    {0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
    {0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
    {0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
    {0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
    {0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
    {0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
    {0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
    {0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
    {0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
    {0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46},
};

//pins configuration
static const int GIO_pin   = 13; //GIO
static const int SDI_pin   = 19; //SDIO
static const int SCLK_pin  = 18; //SCK
static const int CS_pin    = 17; //CS

static const int bind_pin  = 2;
static const int RLED_pin  = 21;
static const int MOT1_pin  = 12;
static const int MOT2_pin  = 11;
static const int MOT3a_pin = 9;
static const int MOT3b_pin = 10;

const uint32_t failsafeTimeout = 200;     //after this time failsafe mode is set
const uint32_t channelChangeTimeout = 30; //after this time next chanel is listen

const uint8_t flashPage[SPM_PAGESIZE] __attribute__ (( aligned(SPM_PAGESIZE) )) PROGMEM = {
    0xAA, 0xE2, 0x35, 0x24, 0x94 //This is WLTOYS transmitter
    //" "
}; //preallocated frame for saving in flash memory id of Tx bytes: (0 to 4)

inline void rLedOff() { digitalWrite(RLED_pin, HIGH);};
inline void rLedOn()  { digitalWrite(RLED_pin, LOW) ;};


//############## GLOBALS ################
static uint8_t ramPage[SPM_PAGESIZE]; //id (0 to 4 bytes) and flash buffer
static uint8_t packet[SPM_PAGESIZE];//at list 21 for storing recived frame

//positions of recived signals (RC channels)
static int16_t actuatorCH[10] = {1500,1500,1500,1500,1500,1500,1500,1500};

//To transfer information from main loop into Interupt routine 
int8_t INTstart=1; //starting index of for array
struct INTdata {
    uint8_t timer;
    int8_t next;
} static INTval[4] = {{0,-1},{0,-1},{0,-1},{0,-1}};

volatile unsigned char adc_lo, adc_hi, o;
volatile uint16_t count;

//############### SETUP ################
void setup() {
    noInterrupts();
    
#if defined(DEBUG)
    Serial.begin(SERIAL_BAUD_RATE);//for debug
#endif

    pinMode(RLED_pin, OUTPUT);
    pinMode(MOT1_pin, OUTPUT);
    pinMode(MOT2_pin, OUTPUT);
    pinMode(MOT3a_pin,OUTPUT);
    pinMode(MOT3b_pin,OUTPUT);
    
    A7105_Init();
    
    if(testBindJumper()) {
        bind_Flysky();    
    }

    optiboot_readPage(flashPage, ramPage, 1);

    //-----------------TIMER2
#if defined __AVR_ATmega8__
    TCCR2  = 0x04; //Prescaler and config
    TIMSK  = 0x40; //Interupt mode bit6
#endif
#if defined __AVR_ATmega88__ | __AVR_ATmega88P__
    TCCR2A = 0x00; //Configure in normal mode (pure increment counting, no PWM etc.)
    TIMSK2 = 0x01; //Disable Compare Match A interrupt enable (only want overflow)
    TCCR2B = 0x04; //Prescaler clock divided by: 1=>1 32kHz, 2=>8 4kHz, 3=>32 1kHz,
                   //      4=>64 500Hz, 5=>128 250Hz, 6=>256 125Hz, 7=>1024 32.5Hz
#endif
    ASSR   = 0x00; //Select clock source: internal I/O clock
    TCNT2  = 0;    //Max time for first count


    //-----------------ADC
    //ADMUX  = 0xe0; //internal source voltage - channel0
    //ADCSRA = 0xfe; //interupt on, free run

    listenNextChannel();
    interrupts();
}

//############ MAIN LOOP ##############
void loop() {
    static uint32_t failsafeTime;

    o=ADCH;
    //Serial.print(adc_hi);Serial.print(" ");Serial.print(adc_lo/64);Serial.print(" ");Serial.println(count);
    count=0;

    if (recivePacket()) {
        //recived
        for (uint8_t i=0; i<8; i++) {
            uint16_t actVal;
            cli();
            actVal=(packet[5+(2*i)]+255*packet[6+(2*i)]);
            sei();
            if ((actVal>900) && (actVal<2200)) {
                actuatorCH[i]=actVal;
            }
#if defined(DEBUG)
            Serial.print(actuatorCH[i]);
            Serial.print(" ");
#endif
        }
        
        
        failsafeTime = millis();
        setActuators();
    } else {
        if( (millis()-failsafeTime) > failsafeTimeout ){
            //Set failsafe actuatorCH
            for (uint8_t i=0; i<8; i++) {
                actuatorCH[i]=1500;
            }
            actuatorCH[2]=1000;
#if defined(DEBUG)
            Serial.print("Failsave! ");
#endif
            failsafeTime = millis();
            setActuators();
        }
    }
}

bool recivePacket(void) {
    //return true in case of correct packet recived
    //it also set listening on correct channel

    static uint32_t lastUpdateTime;


    if (digitalRead(GIO_pin) == LOW) {
        //Packet recived
        if (A7105_checkCRC()) {
            //CRC OK
            Read_Packet();
            if ( (packet[1]==ramPage[1]) && (packet[2]==ramPage[2]) && 
                 (packet[3]==ramPage[3]) && (packet[4]==ramPage[4]) ) {
                //TX ID OK - GOT IT!
                listenNextChannel();
#if defined(DEBUG)
                Serial.print(millis()-lastUpdateTime);
                Serial.print("+ ");
#endif
                lastUpdateTime = millis();
                rLedOn();
                return true;
            }
        }
    }
   
    if ( (millis() - lastUpdateTime) > channelChangeTimeout ) {
          //Times out - action required
          listenNextChannel();
#if defined(DEBUG)
          Serial.print(millis()-lastUpdateTime);
          Serial.println("- ");
#endif
          lastUpdateTime = millis();
          rLedOff();
    }
    return false;
}

void listenNextChannel(){
    static uint8_t chanrow;
    static uint8_t chancol = 0;
    static uint8_t chanoffset;
    static uint8_t channel;
    static bool firstTime = true;

    if (firstTime){
        firstTime = false;
        chanrow=ramPage[1]%16;
        chanoffset=ramPage[1]/16;
        if(chanoffset > 9) chanoffset = 9;//from sloped soarer findings, bug in flysky protocol
    }

    //determine channel
    channel=tx_channels[chanrow][chancol]-chanoffset-1;
    chancol = (chancol + 1) % 16;

#if defined(DEBUG)
    Serial.print(chancol,HEX);
    Serial.print(" ");
    Serial.print(channel,HEX);
    Serial.print("\t");
#endif

    A7105_listenChannel(channel);
}


void setActuators() {
    int16_t motor1;
    int16_t motor2;
    int16_t motor3;
    uint8_t motDir;
    int16_t diff;

    //two motors
    actuatorCH[2]-=1000;
    actuatorCH[0]-=1500;
    
    diff = (actuatorCH[2] / 5) * (actuatorCH[0] / 5) / 80;
    
    motor1 = (actuatorCH[2]+diff)/5;
    motor2 = (actuatorCH[2]-diff)/5;
    
    motor1 = constrain(motor1, 0, 255);
    motor2 = 255-constrain(motor2, 0, 255);

    //setting data (motor 1 and 3) for interupt routine
    if(motor1>motor2) {
        INTstart = ( motor2 == 0 ) ? 2 : 0;
        INTval[0].timer = 255-motor2;
        INTval[2].timer = 255-motor1+motor2;
        INTval[3].timer = motor1;
        INTval[0].next = 2;
        INTval[2].next = ( motor1 == 255 ) ? -1 : 3;
        INTval[3].next = -1;
    }
    if(motor1<motor2) {
        INTstart = ( motor1 == 0 ) ? 1 : 0;
        INTval[0].timer = 255-motor1;
        INTval[1].timer = 255-motor2+motor1;
        INTval[3].timer = motor2;
        INTval[0].next = 1;
        INTval[1].next = ( motor2 == 255 ) ? -1 : 3;
        INTval[3].next = -1;
    }
    if(motor1==motor2) {
        INTval[0].timer = 255-motor1;
        INTval[3].timer = motor1;
        INTval[0].next = 3;
        INTval[3].next = -1;
    }

    //motor3
    motDir = (actuatorCH[1] > 1500) ? LOW : HIGH;
    motor3 = (abs(actuatorCH[1]-1500))/2;
    motor3 = constrain(motor3, 0, 255);
    motor3 = (motor3 < 8) ? 0 : motor3;
    if (motDir) {
        analogWrite(MOT3a_pin, motor3);
        analogWrite(MOT3b_pin, 0);
    } else {
        analogWrite(MOT3a_pin, 0);
        analogWrite(MOT3b_pin, motor3);        
    }
    
#if defined(DEBUG)
    Serial.print(  "m1: ");
    Serial.print(motor1);
    Serial.print("\tm2: ");
    Serial.print(motor2);
    Serial.print("\tm3: ");
    Serial.print(motor3);
    Serial.print("\t");
    Serial.print(motDir);
    Serial.print(" d: ");
    Serial.println(diff);
#endif

}


//BIND_TX
void bind_Flysky() {

    A7105_listenChannel(0);
    
    while(1) {
        //LED blink
        if((millis() % 300 ) > 150) {
            rLedOn();
        } else {
            rLedOff();
        }

        if (digitalRead(GIO_pin) == LOW) {
            if (A7105_checkCRC()) {
                Read_Packet();
                if(packet[0]==0xaa) {
                    optiboot_writePage(flashPage, packet, 1);
                    for(uint8_t i=0; i<5; i++) {
#if defined(DEBUG)
                        Serial.print(ramPage[i]);
#endif
                        ramPage[i]=packet[i];
                    }
#if defined(DEBUG)
                    Serial.println("bind OK");
#endif            
                    return;
                }
            }
            //something went wrong, start listen again
            A7105_listenChannel(0);
        }              
    }//loop
}


//check presence of bind jumper
bool testBindJumper() {
    pinMode(bind_pin, INPUT_PULLUP);//pull up
    if ( digitalRead(bind_pin) == LOW) {
        delayMicroseconds(1);
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

//ADC interupt routine
ISR(ADC_vect) {
    //change channel and load to buffer
    count++;
    adc_hi =  ADCH;
    adc_lo =  ADCL;
}

