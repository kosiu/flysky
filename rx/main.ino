// **********************************************************
// ******************   Flysky Rx Code-PPM  *******************
//               by midelic on RCgroups.com 
//   Thanks to PhracturedBlue,ThierryRC,Dave1993,and the team
//    of OpenLRS project and  Hasi for PPM encoder
// **********************************************************
//Hardware-M8/168/328-8/16Mhz
#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed
#include <EEPROM.h>
//#define DEBUG
#define FAILSAFE
//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the arduino
//////////////////////////////////////////////////////////////////

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];

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

 
 //Dave/mwii pins configuration
 #define GIO_pin 6//GIO-D6
 #define SDI_pin 5 //SDIO-D5 
 #define SCLK_pin 4 //SCK-D4
 #define CS_pin 2//CS-D2
 //#############################
 #define bind A0//bind plug
 #define Servo1_OUT 3 //Servo1(D3)
 #define Servo2_OUT 7 //Servo2(D7)
 #define Servo3_OUT 8 //Servo3(B0)
 #define Servo4_OUT 9 //Servo4(B1)
 #define Servo5_OUT 10 //Servo5(B2)
 #define Servo6_OUT 11 //Servo6(B3)
 #define Servo7_OUT 12 //Servo7(B4)
 #define Servo8_OUT 13 //Servo8(B5)
  //##############################
 #define  CS_on PORTD |= 0x04 //D2
 #define  CS_off PORTD &= 0xFB //D2
 #define  SCK_on PORTD |= 0x10//D4
 #define  SCK_off PORTD &= 0xEF//D4
 #define  SDI_on PORTD |= 0x20 //D5
 #define  SDI_off PORTD &= 0xDF //D5
 #define  GIO_on PORTD |=0x40//D6 
  //#####################################
 #define  GIO_1 (PIND & 0x40) == 0x40 //D6 input
 #define  GIO_0 (PIND & 0x40) == 0x00 //D6
 #define  SDI_1 (PIND & 0x20) == 0x20 //D5
 #define  SDI_0 (PIND & 0x20) == 0x00 //D5
//############################################
 #define RED_LED_pin A3
 #define Red_LED_ON  PORTC |= _BV(3);
 #define Red_LED_OFF  PORTC &= ~_BV(3);
 #define NOP() __asm__ __volatile__("nop")
 //#######################################
 #define Servo1_OUT_HIGH PORTD |= _BV(3) //Servo1(D3)
 #define Servo2_OUT_HIGH PORTD |= _BV(7) //Servo2(D7)
 #define Servo3_OUT_HIGH PORTB |= _BV(0) //Servo3(B0)
 #define Servo4_OUT_HIGH PORTB |= _BV(1) //Servo4(B1)
 #define Servo5_OUT_HIGH PORTB |= _BV(2) //Servo5(B2)
 #define Servo6_OUT_HIGH PORTB |= _BV(3) //Servo6(B3)
 #define Servo7_OUT_HIGH PORTB |= _BV(4) //Servo7(B4)
 #define Servo8_OUT_HIGH PORTB |= _BV(5)//Servo8(B5)
 //#######################################################
 #define Servo_Ports_LOW PORTD &= 0x77; PORTB &= 0xc0;//all servos low
 //***************************************************************************
//For generate PPM signal jumper between servo1 and servo3(pin D3 and pin D8(B0)
//default is with servo output.
//PPm output remain the same on pin D10(B2)
//For binding jumper between A0(C0) and gnd  as any normal rx.
//***************************************************************************
//########## Variables #################
static uint32_t id;
static uint8_t txid[4];
static uint16_t word_temp;
static uint8_t chanrow;
static uint8_t chancol;
static uint8_t chanoffset;
static uint8_t channel;
static uint8_t aid[4];
static uint8_t packet[21];
static uint16_t Servo_data[10] = {1500,1500,1500,1500,1500,1500,1500,1500};
volatile byte scale;
static byte jumper1 = 0;
static byte jumper2 = 0;
static uint16_t total_servo_time=0;
static byte cur_chan_numb=0;


void setup(){//setup()

  pinMode(RED_LED_pin, OUTPUT); 
  //RF module pins
  pinMode(GIO_pin, INPUT);//GIO 1
  pinMode(SDI_pin, OUTPUT);//SDI   SDIO 
  pinMode(SCLK_pin, OUTPUT);//SCLK SCK 
  pinMode(CS_pin, OUTPUT);//CS output
  //####################
  //servo pins
  pinMode(Servo1_OUT, OUTPUT); //Servo1
  pinMode(Servo2_OUT, OUTPUT); //Servo2
  pinMode(Servo3_OUT, OUTPUT); //Servo3
  pinMode(Servo4_OUT, OUTPUT); //Servo4
  //
  pinMode(Servo6_OUT, OUTPUT); //Servo6
  pinMode(Servo7_OUT, OUTPUT); //Servo7
  pinMode(Servo8_OUT, OUTPUT); //Servo8
  //###############
  CS_on;//start CS high
  SDI_on;//start SDIO high
  SCK_off;//start sck low
 # if defined(DEBUG)
 Serial.begin(SERIAL_BAUD_RATE);//for debug 
#endif
  uint8_t i;
  uint8_t if_calibration1;
  uint8_t vco_calibration0;
  uint8_t vco_calibration1;
   
  delay(10);//wait 10ms for A7105 wakeup
  _spi_write_adress(0x00,0x00);//reset A7105
  A7105_WriteID(0x5475c52A);//A7105 id
  #if defined(DEBUG)
  A7105_ReadID();
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
while(_spi_read_adress(0x02)){
if_calibration1=_spi_read_adress(0x22);
if(if_calibration1&0x10){//do nothing
}
}

_spi_write_adress(0x24,0x13);
_spi_write_adress(0x26,0x3b);
_spi_write_adress(0x0F,0x00);//channel 0
_spi_write_adress(0x02,0x02);
while(_spi_read_adress(0x02)){
vco_calibration0=_spi_read_adress(0x25);
if(vco_calibration0&0x08){//do nothing
}
}

_spi_write_adress(0x0F,0xA0);
_spi_write_adress(0x02,0x02);
while(_spi_read_adress(0x02)){
vco_calibration1=_spi_read_adress(0x25);
if(vco_calibration1&0x08){//do nothing
}
}

_spi_write_adress(0x25,0x08);
_spi_strobe(0xA0);//stand-by

//END A7105 init

jumper2=bind_jumper();

while(1){
Red_LED_ON;
delay(500);
Red_LED_OFF;
delay(500);
if (jumper2==0){//bind complete or no bind
uint8_t i;
uint8_t adr=10;
uint8_t x=EEPROM.read(adr);
#if defined(DEBUG)
Serial.print(" ");
Serial.print(x,HEX);
Serial.print(" ");
#endif
if(x==0xaa){
for(i=1;i<5;i++){
txid[i]=EEPROM.read(adr+i);
#if defined(DEBUG)
Serial.print(" ");
Serial.print(txid[i]);
Serial.print(" ");
#endif
}
break;
}
else
continue;
}
else{
bind_Flysky();
Red_LED_ON;
while(1);
}
}

id=(txid[1] | ((uint32_t)txid[2]<<8) | ((uint32_t)txid[3]<<16) | ((uint32_t)txid[4]<<24));

#if defined(DEBUG)
Serial.print(" ");
Serial.print(id,HEX);
#endif
chanrow=id%16;
chanoffset=(id & 0xff) / 16;
chancol=0;
if(chanoffset > 9) chanoffset = 9;//from sloped soarer findings, bug in flysky protocol
// 
#if F_CPU == 16000000// thanks to goebish for this nice idea.
	scale = 2;
#elif F_CPU == 8000000
	scale = 1;
#else
#error // 8 or 16MHz only !
#endif
//
jumper1 = PPM_jumper(); // Check the possible jumper positions for changing the receiver mode.
if(jumper1==1){
//initiallize default ppm values
for(int i=0; i<chanel_number; i++){
   ppm[i]= default_servo_value;
   }
   pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  }  
  OCR1A = 50*scale;
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
 #if defined(__AVR_ATmega8__)
// your m8 timer register code
TIMSK |= (1 << OCIE1A); // enable timer compare interrupt
#elif defined (__AVR_ATmega328P__)
TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
// your m328 timer register code
#else
#error // only m8 & m328 are supported !
#endif 
sei();
}

//############ MAIN LOOP ##############
void loop() {
channel=tx_channels[chanrow][chancol]-chanoffset;
channel-=1;
_spi_strobe(0xA0);
_spi_strobe(0xF0);
_spi_write_adress(0x0F,channel);
_spi_strobe(0xC0);
chancol = (chancol + 1) % 16;
unsigned long pause;
uint8_t x;
pause=micros();
static unsigned long last_rx;
while(1){
#if defined FAILSAFE
      uint8_t n;
      if((millis() - last_rx)>1500){    //fs delay 1500ms
      // enter your fs code here
      Servo_data[2]=1050;  //ER9x, thr min, rest center
      ppm[2] = 1050;
      for (n=0;n<2;n++){
        Servo_data[n]=1500;
        ppm[n] = 1500; }
      for (n=3;n<8;n++){
        Servo_data[n]=1500;
        ppm[n] = 1500; }  
      #if defined(DEBUG)
      Serial.println("failsafe!");
      #endif
      }
   #endif  
if((micros() - pause)>2000){
Red_LED_OFF;
chancol = (chancol + 1) % 16;//advance to the next next packet most likely you missed the next one.
channel=tx_channels[chanrow][chancol]-chanoffset;
channel-=1;
break;
}
if (GIO_1){
continue;
}
x=_spi_read_adress(0x00);
if (!(bitRead(x,5)==0)&& !(bitRead(x,6)==0)){
continue;
}
Read_Packet();
if (!(packet[1]==txid[1])&& !(packet[2]==txid[2])&& !(packet[3]==txid[3])&& !(packet[4]==txid[4])){
continue;
}
Red_LED_ON;
last_rx=millis(); 
uint8_t i;
for (i=0;i<8;i++){
cli();
word_temp=(packet[5+(2*i)]+256*packet[6+(2*i)]);
sei();
if ((word_temp>900) && (word_temp<2200))
Servo_data[i]=word_temp;
ppm[i]=Servo_data[i];
#if defined(DEBUG)
Serial.print(" ");
Serial.print(chancol);
Serial.print(" ");
Serial.print(Servo_data[i]);
Serial.print(" ");
#endif
}
Serial.println(" ");
break;
}
}

ISR(TIMER1_COMPA_vect){
   TCNT1 = 0;
  if (jumper1==0){
  pinMode(Servo5_OUT, OUTPUT); //Servo5
  //static byte cur_chan_numb;
  Servo_Ports_LOW;//all servo ports low
  //code for servo.
  cur_chan_numb++;//next servo
 if(cur_chan_numb < chanel_number){
 total_servo_time +=Servo_data[cur_chan_numb]*scale;
 OCR1A=Servo_data[cur_chan_numb]*scale;
 }
 else{
 OCR1A=PPM_FrLen*scale-total_servo_time;
 cur_chan_numb = 0xff;
 total_servo_time=0;
 }
	  
  switch (cur_chan_numb) {
      case 0:
        Servo1_OUT_HIGH;
        break;
      case 1:
        Servo2_OUT_HIGH;
        break;
      case 2:
        Servo3_OUT_HIGH;
        break;
      case 3:
        Servo4_OUT_HIGH;
        break;
      case 4:
       Servo5_OUT_HIGH;
        break;
      case 5:
        Servo6_OUT_HIGH;
        break;
      case 6:
        Servo7_OUT_HIGH;
        break;
      case 7:
        Servo8_OUT_HIGH;
        break;
  }
  }
else{ 
  static boolean state = true;
   pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
 
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * scale;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);//PPM on servo5
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * scale;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * scale;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
}


//BIND_TX
void bind_Flysky() {
static byte counter1=255;
_spi_strobe(0xA0);
_spi_strobe(0xF0);
_spi_write_adress(0x0F,0x00);//binding listen on channel 0
_spi_strobe(0xC0);
while(counter1){//
delay(10);//wait 10ms
if (bitRead(counter1,2)==1){ 
Red_LED_ON;
}
if(bitRead(counter1,2)==0){
Red_LED_OFF;
}
if (GIO_0){
uint8_t x;
x=_spi_read_adress(0x00);
if ((bitRead(x,5)==0)&&(bitRead(x,6)==0)){//test CRC&CRF bits
Read_Packet();
uint8_t i;
uint8_t adr=10;
for(i=0;i<5;i++){
EEPROM.write(adr+i,packet[i]);
txid[i]=packet[i];
}
break;
}
else{
_spi_strobe(0xA0);
_spi_strobe(0xF0);
_spi_write_adress(0x0F,0x00);//binding listen on channel 0
_spi_strobe(0xC0);//try again
continue;
}
}
else{
--counter1;
if (counter1==0){
counter1=255;
}
}
}
}

unsigned char PPM_jumper(void){
//-- Serial PPM Selection (jumper between Ch1 and ch3)
pinMode(Servo3_OUT, INPUT); //CH3 input
digitalWrite(Servo3_OUT, HIGH); // pull up
digitalWrite(Servo1_OUT, HIGH); // CH1 is HIGH
delayMicroseconds(1);
if ( digitalRead(Servo3_OUT) == HIGH) 
	{
	digitalWrite(Servo1_OUT, LOW); // CH1 is LOW
	delayMicroseconds(1);
	if (digitalRead(Servo3_OUT) == LOW) // OK jumper plugged
			{
                        pinMode(Servo3_OUT, OUTPUT);
			return  1; //Serial PPM OUT
			}
	}
pinMode(Servo3_OUT, OUTPUT);

return  0; // servo PWM by default
}

//bind jumper
unsigned char bind_jumper(void){
pinMode(bind, INPUT_PULLUP);//pull up
if ( digitalRead(bind) == LOW) {
delayMicroseconds(1);
return 1;
}
return  0;
}

