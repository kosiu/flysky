

//this table can be local to function: listenNextChannel() it's here to dont
//blur the code it stores FlySky channels mapping tabele
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


void FlySkyTx::transmit() {
    static uint8_t chanrow;
    static uint8_t chancol;
    static uint8_t chanoffset;
    static uint8_t channel;
    static bool firstTime = true;

    if (firstTime){
        chanrow=id % 16;
        chanoffset=(id & 0xff) / 16;
        if(chanoffset > 9) chanoffset = 9;//from sloped soarer findings, bug in flysky protocol
        chancol=0;
        firstTime=false;
    } else {
        channel=rfChannels[chanrow][chancol]-chanoffset;

        _spi_strobe(0xA0);
        _spi_strobe(0xE0);
        _spi_write_adress(0x0F,channel);
        writePacket(0x55);//was 55 or aa
        _spi_strobe(0xD0);
        chancol = (chancol + 1) % 16;
    }
}

void FlySkyTx::init() {

    A7105_init();

    _spi_read_adress(0x22);
    _spi_write_adress(0x24,0x13);
    _spi_write_adress(0x25,0x09);
    _spi_write_adress(0x28,0x1F);//set power to 1db maximum
    
    id=0x942435e2;//WLTOYS transmmiter
    //randomSeed((uint32_t)analogRead(A0) << 10 | analogRead(A4));
    //id = random(0xfefefefe) + ((uint32_t)random(0xfefefefe) << 16);

    //BIND_TX around 2.5s
    unsigned char counter=255;
    while(counter){
        _spi_strobe(0xA0);
        _spi_strobe(0xE0);
        _spi_write_adress(0x0F,0x01);
        writePacket(0xaa);
        _spi_strobe(0xD0);
        delay(10);

        if (bitRead(counter,3)==1) 
            rLedOn();
        if(bitRead(counter,3)==0)
            rLedOff();
        counter--;
    }
    _spi_strobe(0xA0);//stand-by
}


//--------------------------------------------
//TODO add arguments for budffer
void FlySkyTx::writePacket(uint8_t init) {
    //except adress(0x05)should sent to A7105 21 bytes totally)
    
    CS_off;
    _spi_write(0x05);//TX/RX FIFO adress
    _spi_write(init);//0xaa or 0x55(depend on bind packet or data packet)
    
    _spi_write((id >>  0) & 0xff);
    _spi_write((id >>  8) & 0xff);
    _spi_write((id >> 16) & 0xff);
    _spi_write((id >> 24) & 0xff);
    for(uint8_t i=0; i<8; i++) {
        _spi_write( lowByte(adc.signals[i]));
        _spi_write(highByte(adc.signals[i]));
    }
    CS_on;
}


