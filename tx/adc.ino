
//Analog input config
//ArrayPosition AdcIn What
//            0   5   Right X
//            1   1   Right Y
//            2   0   Left  Y
//            3   3   Left  X / middle trim
//            4   4   Right Pot
//            5   2   Left  Pot

//Global varibles
//volatile variables to transfer data from interupt routines
//it has to be basic data type (dosn't work for arrays and structs)
volatile long int adc0, adc1, adc2, adc3, adc4, adc5;
volatile unsigned char cnt0, cnt1, cnt2, cnt3, cnt4, cnt5;


//ADC interupt routine
ISR(ADC_vect) {

    //change channel and load to buffer
    uint8_t chn = ADMUX & 0b111;

    uint16_t val;
    unsigned char lo;
    lo = ADCL; //this is required (I don't kno why)
    val = (ADCH<<8)+lo;

    switch(chn){
      case 2: if(cnt0 < 255) { adc0 += val; cnt0++;} chn=3; break;
      case 1: if(cnt1 < 255) { adc1 += val; cnt1++;} chn=2; break;
      case 5: if(cnt2 < 255) { adc2 += val; cnt2++;} chn=0; break;
      case 3: if(cnt3 < 255) { adc3 += val; cnt3++;} chn=4; break;
      case 4: if(cnt4 < 255) { adc4 += val; cnt4++;} chn=5; break;
      case 0: if(cnt5 < 255) { adc5 += val; cnt5++;} chn=1; break;
    }
    
    ADMUX  = 0b11000000 | chn;
    ADCSRA |= (1<<ADSC); // start next conversion
    
}
bool Ain::calibration(bool button){
  enum State { firstButtonPress, firstButtonRelease, secondButtonPress };
  static State state = firstButtonPress;

  if((millis() % 64 ) > 32) rLedOn(); else rLedOff();

  memCopy();

  if(button){
    if (state == firstButtonRelease) {
      state =  secondButtonPress;
    }
    
    //set center position
    for (unsigned char i=0; i<6; i++) {
      if (chCfg[i].noSegments == 2) chCfg[i].inPoint[1] = signals[i]/count[i];
    }    
  } else {
    if (state == firstButtonPress) {
      state =  firstButtonRelease;
      //set current values
      for (unsigned char i=0; i<6; i++) {
        int val = signals[i]/count[i];
        chCfg[i].inPoint[0] = val;
        chCfg[i].inPoint[chCfg[i].noSegments] = val;
      }    
    }
    if (state == secondButtonPress) {
      state =  firstButtonPress;
      if (cDbgCal) printChConfig();
      for (unsigned char i=0; i<6; i++) {
        EEPROM.write((i*6+0), (chCfg[i].inPoint[0] >> 0) & 0xff);
        EEPROM.write((i*6+1), (chCfg[i].inPoint[0] >> 8) & 0xff);
        EEPROM.write((i*6+2), (chCfg[i].inPoint[1] >> 0) & 0xff);
        EEPROM.write((i*6+3), (chCfg[i].inPoint[1] >> 8) & 0xff);
        EEPROM.write((i*6+4), (chCfg[i].inPoint[2] >> 0) & 0xff);
        EEPROM.write((i*6+5), (chCfg[i].inPoint[2] >> 8) & 0xff);        
      }
      return false;
    }
    //set minimum maximum
    for (unsigned char i=0; i<6; i++) {
      int val = signals[i]/count[i];
      if ((val - 10 ) > chCfg[i].inPoint[chCfg[i].noSegments]){
        chCfg[i].inPoint[chCfg[i].noSegments] = val - 10;
      } else if ((val + 10 ) < chCfg[i].inPoint[0]){
        chCfg[i].inPoint[0] = val + 10;
      }
    }
  }
  return true;
}

void Ain::printChConfig(){
  for (unsigned char i=0; i<6; i++) {
      Serial.print("Ch ");
      Serial.print(i);
      Serial.print(" ");      
      Serial.print(chCfg[i].isReversed);
      Serial.print(" ");
      Serial.print(chCfg[i].inPoint[0]);
      Serial.print(" ");
      Serial.print(chCfg[i].inPoint[1]);
      Serial.print(" ");
      Serial.println(chCfg[i].inPoint[2]);      
  }
  
}

void Ain::out(){

    memCopy();
     
    //calibration parameters
    if(cDbgAdc) Serial.print("");
    for (unsigned char i=0; i<6; i++) {
      long int outVal;                            
      long int lowLim;
      int segMax = 1000;
      int segMin = 1000;

      if(cDbgAdc) Serial.print(signals[i]/count[i]);
      if(cDbgAdc) Serial.print(" ");
      
      if( signals[i] >= ( (long int)chCfg[i].inPoint[chCfg[i].noSegments] * count[i]) ) {
        outVal=2000;
      } else
      {
        for ( unsigned char s = chCfg[i].noSegments; s > 0; s-- ) {
          segMax = segMin;
          if ( s > 1 ) { segMin = chCfg[i].outPoint[(s-2)]; } else { segMin = 0; }
          if ( signals[i] >= ( lowLim = (long int)chCfg[i].inPoint[s-1] * count[i]) ) {
            
            outVal = signals[i] - lowLim;
            outVal /= ( (long int)(chCfg[i].inPoint[s] - chCfg[i].inPoint[s-1] ) * count[i] ) / (segMax-segMin);
            outVal += 1000 + segMin;
            break;
          }
        }
      }
      if ( signals[i] < ( (long int)chCfg[i].inPoint[0] * count[i]) ) {
        outVal=1000;
      }
      if (chCfg[i].isReversed) outVal = 3000 - outVal;
      signals[i] = outVal; //& 0xfffc;
    };
}

void Ain::memCopy() {
    //copy and clear memory block from ADC interupt routine
    noInterrupts(); signals[0] = adc0; count[0] = cnt0; adc0 = 0; cnt0 = 0; interrupts();
    noInterrupts(); signals[1] = adc1; count[1] = cnt1; adc1 = 0; cnt1 = 0; interrupts();
    noInterrupts(); signals[2] = adc2; count[2] = cnt2; adc2 = 0; cnt2 = 0; interrupts();
    noInterrupts(); signals[3] = adc3; count[3] = cnt3; adc3 = 0; cnt3 = 0; interrupts();
    noInterrupts(); signals[4] = adc4; count[4] = cnt4; adc4 = 0; cnt4 = 0; interrupts();
    noInterrupts(); signals[5] = adc5; count[5] = cnt5; adc5 = 0; cnt5 = 0; interrupts();
}

void Ain::init(){
    ADMUX  = 0b11000000; //internal source voltage - channel0
    ADCSRA = 0b11011100; //interupt on
    ADCSRB = 0b00000000;
}

