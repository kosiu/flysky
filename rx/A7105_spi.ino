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

void Write_Packet(uint8_t init){//except adress(0x05)should sent to A7105 21 bytes totally)
uint8_t i;
CS_off;
_spi_write(0x05);//TX/RX FIFO adress
_spi_write(init);//0xaa or 0x55(depend on bind packet or data packet)
_spi_write((id >>  0) & 0xff);
_spi_write((id >>  8) & 0xff);
_spi_write((id >>  16) & 0xff);
_spi_write((id >>  24) & 0xff);

for(i=0;i<8;i++){
cli();
packet[0+2*i]=lowByte(Servo_data[i]);//low byte of servo timing(1000-2000us)
packet[1+2*i]=highByte(Servo_data[i]);//high byte of servo timing(1000-2000us)
sei();
_spi_write(packet[0+2*i]);
_spi_write(packet[1+2*i]);
}

CS_on;
}

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

