// **********************************************************
// ******************   Flysky Rx Code-PPM  *******************
//               by midelic on RCgroups.com 
//   Thanks to PhracturedBlue,ThierryRC,Dave1993,the team
//    of OpenLRS project and  Hasi for PPM encoder
//  Added PPM encoder and tested by Philip Cowzer(Sadsack) 
// **********************************************************

#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed
#include <EEPROM.h>

#define FAILSAFE
//#define DEBUG

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 11  //set PPM signal output pin on the arduino  
//Cheapduino: D11
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
 
 
//Cheapduino
#define GIO_pin 9     //GIO - D9 - PB1
#define SDI_pin 10    //SDIO - D10 - PB2 
#define SCLK_pin A4   //SCK - A4 - PC4
#define CS_pin A0     //CS - A0 - PC0
//------------------
#define  CS_on PORTC |= 0x01 //A0
#define  CS_off PORTC &= 0xFE //A0
#define  SCK_on PORTC |= 0x10//A4
#define  SCK_off PORTC &= 0xEF//A4
#define  SDI_on PORTB |= 0x04 //D10 
#define  SDI_off PORTB &= 0xFB //D10 
#define  GIO_on PORTB |=0x02//D9 
//------------------------------------
#define  GIO_1 (PINB & 0x02) == 0x02 //D9 input
#define  GIO_0 (PINB & 0x02) == 0x00 //D9
#define  SDI_1 (PINB & 0x04) == 0x04 //D10
#define  SDI_0 (PINB & 0x04) == 0x00 //D10
//----------------------------------------

#define RED_LED_pin 13 //std arduino LED
#define Red_LED_ON  PORTB |= _BV(5);
#define Red_LED_OFF  PORTB &= ~_BV(5);
#define NOP() __asm__ __volatile__("nop")
  
//########## Variables #################
static uint32_t id;
static uint8_t txid[4];
static uint16_t word_temp;
static uint8_t chanrow;
static uint8_t chancol;
static uint8_t chanoffset;
static uint8_t channel;
static word counter1=512;
static uint8_t aid[4];
static uint8_t packet[21];
static uint16_t Servo_data[10] = {1500,1500,1500,1500,1500,1500,1500,1500};

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest)* 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen)* 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

void setup(){//setup()

  //pinMode(GREEN_LED_pin, OUTPUT);;  
  pinMode(RED_LED_pin, OUTPUT); 
  //RF module pins
  pinMode(GIO_pin, INPUT);//GIO 1
  pinMode(SDI_pin, OUTPUT);//SDI   SDIO 
  pinMode(SCLK_pin, OUTPUT);//SCLK SCK 
  pinMode(CS_pin, OUTPUT);//CS output
  CS_on;//start CS high
  SDI_on;//start SDIO high
  SCK_off;//start sck low
#if defined(DEBUG)
  Serial.begin(SERIAL_BAUD_RATE);//for debug 
#endif
  uint8_t i;
  uint8_t if_calibration1;
  uint8_t vco_calibration0;
  uint8_t vco_calibration1;
  delay(10);//wait 10ms for A7105 wakeup
  _spi_write_adress(0x00,0x00);//reset A7105
  A7105_WriteID(0x5475c52A);//A7105 id
  A7105_ReadID();
#if defined(DEBUG)  
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
bind_Flysky();
id=(txid[0] | ((uint32_t)txid[1]<<8) | ((uint32_t)txid[2]<<16) | ((uint32_t)txid[3]<<24));
#if defined(DEBUG) 
Serial.print(" ");
Serial.print(id,HEX);
#endif
chanrow=id%16;
chanoffset=(id & 0xff) / 16;
chancol=0;
if(chanoffset > 9) chanoffset = 9;//from slope soarer findings, bug in flysky protocol
//initiallize default ppm values
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
  }
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 1 microseconds at 8 mhz
  TIMSK |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

//############ MAIN LOOP ##############
void loop() {
channel=tx_channels[chanrow][chancol]-1-chanoffset;
_spi_strobe(0xA0);
_spi_strobe(0xF0);
_spi_write_adress(0x0F,channel);
_spi_strobe(0xC0);
chancol = (chancol + 1) % 16;
unsigned long pause;
#if defined FAILSAFE
static unsigned long last_rx;
#endif
uint8_t x;
pause=micros();
while(1){

//failsafe
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
chancol = (chancol + 1) % 16;
channel=tx_channels[chanrow][chancol]-1-chanoffset;
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
if (!(packet[1]==txid[0])&& !(packet[2]==txid[1])&& !(packet[3]==txid[2])&& !(packet[4]==txid[3])){
continue;
}
Red_LED_ON;
#if defined FAILSAFE
last_rx=millis(); //reset failsafe timer
#endif
uint8_t i;
cli();
for (i=0;i<8;i++){
word_temp=(packet[5+(2*i)]+256*packet[6+(2*i)]);
if ((word_temp>900) && (word_temp<2200))
Servo_data[i]=word_temp;
ppm[i]=Servo_data[i];// <<< Added By Philip Cowzer AKA 'SadSack' and few deletions with Lots of Cuts&Pastes! Arduino IDE is pony
#if defined(DEBUG)
Serial.print(" ");
Serial.print(Servo_data[i]);
#endif
sei();
}
#if defined(DEBUG)
Serial.println();
#endif
break;
}
}


//BIND_TX
void bind_Flysky() {
uint8_t flag=0;
while(1){
_spi_strobe(0xA0);
_spi_strobe(0xF0);
_spi_write_adress(0x0F,0x00);//binding listen on channel 0
_spi_strobe(0xC0);
while(counter1){//counter for 5 sec.
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
for(i=0;i<4;i++){
EEPROM.write(adr,packet[i+1]);
adr=adr+1;
txid[i]=packet[i+1];
}
break;
}
else{
flag=1;
break;
    }
}
counter1--;
if(counter1){
continue;
}
else{
uint8_t i;
uint8_t adr=10;
for(i=0;i<4;i++){
txid[i]=EEPROM.read(adr);
adr=adr+1;
}
break;
}
}
if(flag==1)
continue;
break;
}
}

//-------------------------------
//-------------------------------
//A7105 SPI routines
//-------------------------------
//-------------------------------
void A7105_WriteID(uint32_t ida) {
 CS_off;
_spi_write(0x06);
_spi_write((ida>>24)&0xff); 
_spi_write((ida>>16)&0xff);
_spi_write((ida>>8)&0xff);
_spi_write((ida>>0)&0xff);
 CS_on;
}
void A7105_ReadID(){
uint8_t i;
 CS_off;
_spi_write(0x46);
for(i=0;i<4;i++){
aid[i]=_spi_read();
}
CS_on;
}
//----------------------
void Read_Packet() {
uint8_t i;
CS_off;
_spi_write(0x45);
for (i=0;i<21;i++) {
packet[i]=_spi_read();
}
CS_on;
}
//---------------------------------
//-------------------------------------- 
void _spi_write(uint8_t command) {  
  uint8_t n=8; 
  SCK_off;//SCK starts low
  SDI_off;
  while(n--) {
    if(command&0x80)
      SDI_on;
    else 
     SDI_off;
     SCK_on;
	 NOP();
	 SCK_off;
    command = command << 1;
  }
  SDI_on;
}  
void _spi_write_adress(uint8_t address, uint8_t data) {
   CS_off;
  _spi_write(address); 
   NOP();
  _spi_write(data);  
   CS_on;
} 
//-----------------------------------------
  uint8_t _spi_read(void) {
  uint8_t result;
  uint8_t i;
  result=0;
  pinMode(SDI_pin,INPUT);//make SDIO pin input
  //SDI_on;
  for(i=0;i<8;i++) {                    
	if(SDI_1)  //if SDIO ==1 
      result=(result<<1)|0x01;
	  else
	  result=result<<1;
    SCK_on;
    NOP();
	SCK_off;
	NOP();
  }
  pinMode(SDI_pin,OUTPUT);//make SDIO pin output again
  return result;
  }   
//--------------------------------------------
uint8_t _spi_read_adress(uint8_t address) { 
  uint8_t result;
  CS_off;
  address |=0x40;
  _spi_write(address);
  result = _spi_read();  
  CS_on;
  return(result); 
} 
//------------------------
void _spi_strobe(uint8_t address) {
 CS_off;
_spi_write(address);
 CS_on;
}
//------------------------
void A7105_reset(void) {
  _spi_write_adress(0x00,0x00); 
}

