
// **********************************************************
// ******************    Flysky Rx Code   *******************
//               by midelic on RCgroups.com
//   Thanks to PhracturedBlue,ThierryRC,Dave1993,and the team
//    of OpenLRS project and  Hasi for PPM encoder
//                 Modified by Jacek Kosek
// **********************************************************
#include <optiboot.h>
#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed


//pins configuration
//Digital In
static const int tlb_pin  = 7;
static const int tlf_pin  = 8;
static const int flu_pin  = 6;
static const int fru_pin  = 5;
static const int frd_pin  = 21;
//Digital Out
static const int wled_pin = 2;
static const int rled_pin = 3;

inline void rLedOff() { digitalWrite(rled_pin, HIGH);};
inline void rLedOn()  { digitalWrite(rled_pin, LOW) ;};
inline void wLedOff() { digitalWrite(wled_pin, HIGH);};
inline void wLedOn()  { digitalWrite(wled_pin, LOW) ;};


class Adc {
  public:
    void out(); //return calculated analog positions 1000 to 2000
    void out2();
    void init();
    void memCopy();
    long int signals[6];
    unsigned char count[6];
} adc;

//############### SETUP ################
void setup() {
    noInterrupts();
    
    Serial.begin(SERIAL_BAUD_RATE);//for debug

    pinMode(rled_pin, OUTPUT);
    pinMode(wled_pin, OUTPUT);

    pinMode(tlb_pin, INPUT_PULLUP);
    pinMode(tlf_pin, INPUT_PULLUP);
    pinMode(flu_pin, INPUT_PULLUP);
    pinMode(fru_pin, INPUT_PULLUP);
    pinMode(frd_pin, INPUT_PULLUP);

    adc.init();

    interrupts();
}

//############ MAIN LOOP ##############
void loop() {
  
    
    //adc.memCopy();
    adc.out();
    
    for (int i=0; i<6; i++) {
        Serial.print(adc.count[i]);
        Serial.print(" ");      
        //Serial.print(adc.signals[i]/adc.count[i]);
        Serial.print(adc.signals[i]);
        Serial.print(" ");
    }
    
    
    digitalWrite(rled_pin,digitalRead(tlf_pin));
    digitalWrite(wled_pin,digitalRead(fru_pin));
    Serial.println("");
    
    delay(100);
}

static const uint8_t A7105_regs[] = {
    0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff ,0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
    0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f, 0xff,
};
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

 
//Spi Comm.pins with A7105/PPM
  #define PPM_pin 3//PPM in 
  #define SDI_pin 5 //SDIO-D5 
  #define SCLK_pin 4 //SCK-D4
  #define CS_pin 2//CS-D2
  //---------------------------------  
  #define  CS_on PORTD |= 0x04 //D2
  #define  CS_off PORTD &= 0xFB //D2
  //
  #define  SCK_on PORTD |= 0x10//D4
  #define  SCK_off PORTD &= 0xEF//D4
  #define  SDI_on PORTD |= 0x20 //D5
  #define  SDI_off PORTD &= 0xDF //D5
  //
  #define  SDI_1 (PIND & 0x20) == 0x20 //D5
  #define  SDI_0 (PIND & 0x20) == 0x00 //D5
  //
  #define RED_LED_pin 13//promini LED
  #define Red_LED_ON  PORTB |= _BV(5);//promini status LED on B5
  #define Red_LED_OFF  PORTB &= ~_BV(5);
  #define NOP() __asm__ __volatile__("nop")
  
//########## Variables #################
static uint32_t id;//tx id, don't confuse it with A7105 id
static uint8_t chanrow;
static uint8_t chancol;
static uint8_t chanoffset;
static uint8_t channel;
static word byte=255;
static uint8_t aid[4];//for debug only
static uint8_t packet[21];
volatile uint16_t Servo_data[10] = {1500,1500,1000,1500,1500,1500,1500,1500};//8 channels

void setup() {
  pinMode(RED_LED_pin, OUTPUT); 
  //RF module pins
  pinMode(PPM_pin, INPUT);//PPM input
  pinMode(SDI_pin, OUTPUT);//SDI   SDIO 
  pinMode(SCLK_pin, OUTPUT);//SCLK SCL 
  pinMode(CS_pin, OUTPUT);//CS output
  CS_on;//start CS high
  SDI_on;//start SDIO high
  SCK_off;//start sck low
 //PPm setup 
attachInterrupt(PPM_pin - 2, read_ppm, CHANGE);
TCCR1A = 0;
TCCR1B = 0;
TCCR1B |= (1 << CS11); 
 // 
  
  #if defined(DEBUG)
  Serial.begin(SERIAL_BAUD_RATE);//for debugging
  #endif
  uint8_t i;
  uint8_t if_calibration1;
  uint8_t vco_calibration0;
  uint8_t vco_calibration1;
 
  //for debug 
  delay(10);//wait 10ms for A7105 wakeup
  _spi_write_adress(0x00,0x00);
  A7105_WriteID(0x5475c52A);//0x2Ac57554
 #if defined(DEBUG)
  A7105_ReadID();//for debug only
  Serial.print(aid[0],HEX);
  Serial.print(aid[1],HEX);
  Serial.print(aid[2],HEX);
  Serial.print(aid[3],HEX);
 #endif 
  for (i = 0; i < 0x33; i++){
        if(A7105_regs[i] != 0xff)
            _spi_write_adress(i, A7105_regs[i]);

}
_spi_strobe(0xA0);//stand-by
_spi_write_adress(0x02,0x01);
while(1){
if_calibration1=_spi_read_adress(0x02);
if(if_calibration1==0){
break;
}
}
_spi_read_adress(0x22);
_spi_write_adress(0x24,0x13);
_spi_write_adress(0x25,0x09);
_spi_write_adress(0x28,0x1F);//set power to 1db maximum
_spi_strobe(0xA0);//
//id=0x30000006;//fixed TX ID(Thierry ID)
//id=0x92040020;
randomSeed((uint32_t)analogRead(A0) << 10 | analogRead(A4));
id = random(0xfefefefe) + ((uint32_t)random(0xfefefefe) << 16);
#if defined(DEBUG)
Serial.print(" ");
Serial.print(id,HEX);
Serial.print(" ");
#endif

bind_Flysky();
Red_LED_ON;
chanrow=id % 16;
chanoffset=(id & 0xff) / 16;
chancol=0;
}

//servodata timing range for flysky.
////-100% =~ 0x03e8//=1000us(min)
//+100% =~ 0x07ca//=1994us(max)
//Center = 0x5d9//=1497us(center)
//channel order AIL;ELE;THR;RUD;AUX1;AUX2;AUX3;AUX4

//############ MAIN LOOP ##############
void loop() {
unsigned long pause;
pause=micros();
_spi_strobe(0xE0);
Write_Packet(0x55);//
channel=tx_channels[chanrow][chancol]-chanoffset;
_spi_write_adress(0x0F,channel);
_spi_strobe(0xD0);
chancol = (chancol + 1) % 16;
while(micros()-pause<1460);
}


void read_ppm() {
	static unsigned int pulse;
	static unsigned long counterPPM;
	static byte chan;
	counterPPM = TCNT1;
	TCNT1 = 0;
#if F_CPU == 16000000//thanks to goebisch for this one
	const long scale = 2;
#elif F_CPU == 8000000
	const long scale = 1;
#else
#error // 8 or 16MHz only !
#endif
	if(counterPPM < 510*scale) {  //must be a pulse if less than 510us
		pulse = counterPPM;
	}
	else if(counterPPM > 1910*scale) {  //sync pulses over 1910us
		chan = 0;
	}
	else{  //servo values between 510us and 2420us will end up here
		Servo_data[chan]= (counterPPM + pulse)/scale;
		chan++;
	}
}

//BIND_TX
void bind_Flysky() {
while(counter){
_spi_strobe(0xA0);
_spi_strobe(0xE0);
_spi_write_adress(0x0F,0x01);
Write_Packet(0xaa);
_spi_strobe(0xD0);
delay(10);
if (bitRead(counter,3)==1) 
Red_LED_ON;
if(bitRead(counter,3)==0)
Red_LED_OFF;
counter--;
}
}


