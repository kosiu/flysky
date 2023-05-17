

//this table can be local to function: listenNextChannel() it's here to dont blur the code
//it stores FlySky channels mapping tabele
const uint8_t rfChannels[16][16] = {
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
    {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46}
};

//--------------------------------BIND
void FlySkyRx::bind() {

  A7105_listenChannel(0);

  if (cDbgHw) Serial.println("Binding");

  while(1) {

    //LED fast blink
    if((millis() % 150 ) > 75) rLedOn(); else rLedOff();

    if (digitalRead(GIO_pin) == LOW) {
      if (A7105_checkCRC()) {
        if ( readPacket() == 0xaa ) {
          //uint8_t ramPage[SPM_PAGESIZE]; //id (0 to 4 bytes) and flash buffer
          id = idReceived;
          Serial.println(id,HEX);
          Serial.println(" <- id");
          EEPROM.write(0, (id >>  0) & 0xff);
          EEPROM.write(1, (id >>  8) & 0xff);
          EEPROM.write(2, (id >> 16) & 0xff);
          EEPROM.write(3, (id >> 24) & 0xff);

          if (cDbgHw) Serial.println(fromEEPROM(0),HEX);
          if (cDbgHw) Serial.println("bind OK");
          return;
        }
      }
      //something went wrong, start listen again
      A7105_listenChannel(0);
    }
  }//loop
}




//--------------------------------
bool FlySkyRx::received(void) {
    //return true in case of correct packet recived
    //it also set listening on correct channel

    static uint32_t lastUpdateTime;

    if (digitalRead(GIO_pin) == LOW) {
        //Some packet recived
        if (A7105_checkCRC()) {
            //with CRC OK
            readPacket();
            if ( id == idReceived ) {
                //TX ID OK - GOT IT!
                listenNext();
                if(cDbgIn) {
                  Serial.print(millis()-lastUpdateTime);
                  Serial.print("+ ");
                }
                lastUpdateTime = millis();
                rLedOn();
                return true;
            }
        }
    }

    if ( (millis() - lastUpdateTime) > channelChangeTimeout ) {
          //Times out - action required
          listenNext();
          if(cDbgIn) {
            Serial.print(millis()-lastUpdateTime);
            Serial.println("- ");
          }
          lastUpdateTime = millis();
          rLedOff();
    }
    return false;
}



//--------------------------------
void FlySkyRx::listenNext(){
    static uint8_t chanrow;
    static uint8_t chancol = 0;
    static uint8_t chanoffset;
    static uint8_t channel;
    static bool firstTime = true;

    if (firstTime){
        firstTime = false;
        chanrow=id%16;
        chanoffset=id/16;
        if(chanoffset > 9) chanoffset = 9;//from sloped soarer findings, bug in flysky protocol
    }

    //determine channel
    channel = rfChannels[chanrow][chancol]-chanoffset-1;
    chancol = (chancol + 1) % 16;

    if( cDbgIn ) {
      Serial.print(chancol,HEX);
      Serial.print(" ");
      Serial.print(channel,HEX);
      Serial.print("\t");
    }
    A7105_listenChannel(channel);
}



//--------------------------------
void FlySkyRx::init() {

    A7105_init();

    _spi_write_adress(0x24,0x13);
    _spi_write_adress(0x26,0x3b);

    _spi_write_adress(0x0F,0x00);//channel 0
    _spi_write_adress(0x02,0x02);

    //this calibrations are never used
    uint8_t vco_calibration0;
    while(_spi_read_adress(0x02)) {
        vco_calibration0=_spi_read_adress(0x25);
        if(vco_calibration0 & 0x08) { //do nothing
        }
    }

    _spi_write_adress(0x0F,0xA0);
    _spi_write_adress(0x02,0x02);

    //this calibrations are never used
    uint8_t vco_calibration1;
    while(_spi_read_adress(0x02)) {
        vco_calibration1=_spi_read_adress(0x25);
        if(vco_calibration1 & 0x08) { //do nothing
        }
    }

    _spi_write_adress(0x25,0x08);
    _spi_strobe(0xA0);//stand-by
}


//--------------------------------------------
unsigned char FlySkyRx::readPacket() {
    unsigned char type;

    //idReceived = 0;
    CS_off;
    _spi_write(0x45);
    type = _spi_read();
    idReceived  = (uint32_t)_spi_read() <<  0;
    idReceived += (uint32_t)_spi_read() <<  8;
    idReceived += (uint32_t)_spi_read() << 16;
    idReceived += (uint32_t)_spi_read() << 24;

    for (uint8_t i=0; i<8; i++) {
        data[i]  = _spi_read() << 0;
        data[i] += _spi_read() << 8;
    }
    CS_on;
    return type;
}

