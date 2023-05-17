
#define NOP() __asm__ __volatile__("nop")

//macros for simplification and convinience
#define CS_on    digitalWrite(CS_pin,  HIGH)
#define CS_off   digitalWrite(CS_pin,  LOW)
#define SCK_on   digitalWrite(SCLK_pin,HIGH)
#define SCK_off  digitalWrite(SCLK_pin,LOW)
#define SDI_on   digitalWrite(SDI_pin, HIGH)
#define SDI_off  digitalWrite(SDI_pin, LOW)

static const uint8_t A7105_regs[] = {
    0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff ,0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
    0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f, 0xff
};


//-------------------------------
//-------------------------------
//A7105 routines
//-------------------------------
//-------------------------------

void A7105_init() {
  //this calibrations are never used
  uint8_t if_calibration1;

  //RF module pins
  pinMode(GIO_pin, INPUT);
  pinMode(SDI_pin, OUTPUT);
  pinMode(SCLK_pin,OUTPUT);
  pinMode(CS_pin,  OUTPUT);

  CS_on;  //start CS high
  SDI_on; //start SDIO high
  SCK_off;//start sck low

  delay(10);//wait 10ms for A7105 wakeup
  A7105_reset();
  A7105_WriteID(0x5475c52A);//A7105 id 42 94 95 22 34


  if (cDbgHw) Serial.println(A7105_ReadID(),HEX);
  
  //Writing all registers
  for (uint8_t i = 0; i < 0x33; i++) {
      if(A7105_regs[i] != 0xff)
          _spi_write_adress(i, A7105_regs[i]);
  }
  
  _spi_strobe(0xA0);//stand-by
  _spi_write_adress(0x02,0x01);

  while(_spi_read_adress(0x02)) {
      if_calibration1=_spi_read_adress(0x22);
      if(if_calibration1&0x10) { //do nothing
      }
  }
  
}
//-------------------------------
bool A7105_checkCRC() {
    uint8_t x;
    x=_spi_read_adress(0x00);
    if ((bitRead(x,5)==0)&&(bitRead(x,6)==0)) { //test CRC&CRF bits
        return true;
    } else {
        return false;
    }
}
//-------------------------------
void A7105_listenChannel(uint8_t channel) {
    _spi_strobe(0xA0);
    _spi_strobe(0xF0);
    _spi_write_adress(0x0F,channel);
    _spi_strobe(0xC0);
}
//-------------------------------
void A7105_WriteID(uint32_t ida) {
    CS_off;
    _spi_write(0x06);
    _spi_write( (ida>>24) & 0xff );
    _spi_write( (ida>>16) & 0xff );
    _spi_write( (ida>> 8) & 0xff );
    _spi_write( (ida>> 0) & 0xff );
    CS_on;
}
//-------------------------------
uint32_t A7105_ReadID() {
    uint32_t ida = 0;
    CS_off;
    _spi_write(0x46);
    ida += ((uint32_t)_spi_read() << 24);
    ida += ((uint32_t)_spi_read() << 16);
    ida += ((uint32_t)_spi_read() <<  8);
    ida += ((uint32_t)_spi_read() <<  0);
    CS_on;
    return ida;
}
//------------------------
void A7105_reset() {
    _spi_write_adress(0x00,0x00);
}



//--------------------------------------------
//--------------------------------------------
// SPI routines
//--------------------------------------------
//--------------------------------------------
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
//--------------------------------------------
void _spi_write_adress(uint8_t address, uint8_t data) {
    CS_off;
    _spi_write(address);
    NOP();
    _spi_write(data);
    CS_on;
}
//-----------------------------------------
uint8_t _spi_read() {
    uint8_t result;
    uint8_t i;
    result=0;
    pinMode(SDI_pin,INPUT);//make SDIO pin input
    //SDI_on;
    for(i=0; i<8; i++) {
        if(digitalRead(SDI_pin) == HIGH)  //if SDIO ==1
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
//--------------------------------------------
void _spi_strobe(uint8_t address) {
    CS_off;
    _spi_write(address);
    CS_on;
}




