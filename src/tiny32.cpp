/***********************************************************************
 * File         :     tiny32.cpp
 * Description  :     Class for Hardware config and function for blynkide32_v2 module
 * Author       :     Tenergy Innovation Co., Ltd.
 * Date         :     23 Nov 2021
 * website      :     http://www.tenergyinnovation.co.th
 * Email        :     uten.boonliam@innovation.co.th
 * TEL          :     089-140-7205
 ***********************************************************************/

#include "tiny32.h"
#include "Arduino.h"
#include "Ticker.h"
#include "tiny32_Lib.h"

Ticker tickerRedLED;
Ticker tickerBlueLED;
Ticker tickerBuilinLED;

//rs485
HardwareSerial rs485(1);
HardwareSerial rs485_2(1);


tiny32::tiny32(){

    pinMode(SW1,INPUT);
    pinMode(SW2,INPUT);
    pinMode(SLID_SW,INPUT);
    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(LED_D1,OUTPUT);
    pinMode(LED_D5,OUTPUT);
    pinMode(RELAY,OUTPUT);
    pinMode(BUZZER,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
    digitalWrite(LED_D1,LOW);
    digitalWrite(LED_D5,LOW);
    digitalWrite(BUZZER,LOW);
}



 /***********************************************************************
 * FUNCTION:    library_version
 * DESCRIPTION: Print out library version
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void tiny32::library_version(void) 
{ 
    Serial.printf("library version: %s\r\n",version_c); 
}

 /***********************************************************************
 * FUNCTION:    Relay
 * DESCRIPTION: Control ON-OFF Relay[pin25]
 * PARAMETERS:  0 or 1
 * RETURNED:    nothing
 ***********************************************************************/
void tiny32::Relay(bool state){
    digitalWrite(RELAY,state);
}

 /***********************************************************************
 * FUNCTION:    RedLED
 * DESCRIPTION: ON-OFF Red LED[pin4]
 * PARAMETERS:  0 or 1
 * RETURNED:    nothing
 ***********************************************************************/
void tiny32::BlueLED(bool state){
    digitalWrite(LED_D5,state);
}

 /***********************************************************************
 * FUNCTION:    BlueLED
 * DESCRIPTION: ON-OFF Blue LED[pin12]
 * PARAMETERS:  0 or 11
 * RETURNED:    nothing
 ***********************************************************************/
void tiny32::RedLED(bool state){
    digitalWrite(LED_D1,state);
}


 /***********************************************************************
 * FUNCTION:    BlueinLED
 * DESCRIPTION: ON-OFF Bulidin LED[pin2]
 * PARAMETERS:  0 or 1
 * RETURNED:    nothing
 ***********************************************************************/
void tiny32::BuildinLED(bool state){
    digitalWrite(LED_BUILTIN,state);
}

 /***********************************************************************
 * FUNCTION:    buzzer_beep
 * DESCRIPTION: buzzer sound beep[pin13]
 * PARAMETERS:  times
 * RETURNED:    nothing
 ***********************************************************************/
void tiny32::buzzer_beep(int times){
  #define PLUSE_BUZZER
//#define DC_BUZZER
    
  for(int _i=0; _i < times; _i++){
        #ifdef PLUSE_BUZZER
        #define dt  50 //0.1sec

         int _j;
         for(_j = dt ; _j > 0; _j--){
               digitalWrite(BUZZER,HIGH);
               vTaskDelay(1);
               digitalWrite(BUZZER,LOW);
               vTaskDelay(1);

         }
         vTaskDelay(100);
        #endif

        #ifdef DC_BUZZER
            digitalWrite(BUZZER,HIGH);
            vTaskDelay(70);
            digitalWrite(BUZZER,LOW);
            vTaskDelay(70);
        #endif  
  }
}  


 /***********************************************************************
 * FUNCTION:    Sw1
 * DESCRIPTION: Read SW1[pin33]
 * PARAMETERS:  nothing
 * RETURNED:    0 or 1
 ***********************************************************************/
bool tiny32::Sw1(void){
    bool _status = !digitalRead(SW1);
    return _status;
}

 /***********************************************************************
 * FUNCTION:    Sw2
 * DESCRIPTION: Read SW2[pin14]
 * PARAMETERS:  nothing
 * RETURNED:    0 or 1
 ***********************************************************************/
bool tiny32::Sw2(void){
    bool _status = !digitalRead(SW2);
    return _status;
}

 /***********************************************************************
 * FUNCTION:    Slid_sw
 * DESCRIPTION: Read Slid Switch[15]
 * PARAMETERS:  nothing
 * RETURNED:    0 or 1
 ***********************************************************************/
bool tiny32::Slid_sw(void){
    bool _status = !digitalRead(SLID_SW);
    return _status;
}


 /***********************************************************************
 * FUNCTION:    TickBlueLED
 * DESCRIPTION: Blink BlueLED[pin4]
 * PARAMETERS:  nothing
 * RETURNED:    0 = Off, 1 < blink 
 ***********************************************************************/
void tiny32::TickBlueLED(float second){
    if(second<=0){
        tickerBlueLED.detach();
        BlueLED(0);
    }
    else{
        tickerBlueLED.attach(second,TickBlueLED_blink);
    }
}

 /***********************************************************************
 * FUNCTION:    TickRedLED
 * DESCRIPTION: Blink RedLED[pin12]
 * PARAMETERS:  nothing
 * RETURNED:    0 = Off, 1 < blink 
 ***********************************************************************/
void tiny32::TickRedLED(float second)
{
    if(second<=0){
        tickerRedLED.detach();
        RedLED(0);
    }
    else{
        tickerRedLED.attach(second,TickRedLED_blink);
    }
}

 /***********************************************************************
 * FUNCTION:    BuildinLED
 * DESCRIPTION: Blink BuildinLED[pin2]
 * PARAMETERS:  nothing
 * RETURNED:    0 = Off, 1 < blink 
 ***********************************************************************/
void tiny32::TickBuildinLED(float second) 
{
    if(second<=0){
        tickerBuilinLED.detach();
        BuildinLED(0);
    }
    else{
        tickerBuilinLED.attach(second,TickBuildinLED_blink);
    }
}

 /***********************************************************************
 * FUNCTION:    PWN_Setup
 * DESCRIPTION: Pulse Width Mod setting up
 * PARAMETERS:  channel[0-15], frequency(Hz), resolution[1-15], pin
 * RETURNED:    0 = error, 1 = pass 
 ***********************************************************************/
bool tiny32::PWM_Setup(uint8_t channel, double freq, uint8_t resolution_bit, uint8_t pin){
        if(channel>15 || channel<0 || resolution_bit>15 || resolution_bit<1)
        {
            Serial.printf("Error: wrong parameter!!\r\n");
            return 0;
        }
        ledcSetup(channel,freq,resolution_bit);
        ledcAttachPin(pin,channel);
        _resolution_bit = resolution_bit;
        return 1;
}

 /***********************************************************************
 * FUNCTION:    PWM_Drive
 * DESCRIPTION: Pulse Width Mod drive
 * PARAMETERS:  channel[0-15], percentage[0-100]
 * RETURNED:    0 = error, 1 = pass 
 ***********************************************************************/
bool tiny32::PWM_Drive(uint8_t channel,uint8_t percentage){
    double _duty=0;

    if(percentage>100 || percentage<0)
    {
        Serial.printf("Error: Out of range parameter\r\n");
        return 0;
    }
    else
    {
        // Serial.printf("_resolution_bit = %d\r\n",_resolution_bit);
        // Serial.printf("percentage = %d\r\n",percentage);

        if(_resolution_bit==1) _duty = (double)percentage*0.02;
        else if(_resolution_bit==2) _duty = (double)percentage*0.04;
        else if(_resolution_bit==3) _duty = (double)percentage*0.08;
        else if(_resolution_bit==4) _duty = (double)percentage*0.16;
        else if(_resolution_bit==5) _duty = (double)percentage*0.32;
        else if(_resolution_bit==6) _duty = (double)percentage*0.64;
        else if(_resolution_bit==7) _duty = (double)percentage*1.28;
        else if(_resolution_bit==8) _duty = (double)percentage*2.56;
        else if(_resolution_bit==9) _duty = (double)percentage*5.12;
        else if(_resolution_bit==10) _duty = (double)percentage*10.24;
        else if(_resolution_bit==11) _duty = (double)percentage*20.48;
        else if(_resolution_bit==12) _duty = (double)percentage*40.96;
        else if(_resolution_bit==13) _duty = (double)percentage*81.92;
        else if(_resolution_bit==14) _duty = (double)percentage*163.84;
        else if(_resolution_bit==15) _duty = (double)percentage*327.68;

        ledcWrite(channel,_duty);
        // Serial.printf("Debug: duty = %f\r\n",_duty);
        return 1;   
    }
}

/***********************************************************************
 * FUNCTION:    TimeStamp_minute_encode
 * DESCRIPTION: number of days since 2000/01/01, valid for 2001..2099
 * PARAMETERS:  y, m, d, h, mi (Example: 2020,8,13,22,55)
 * RETURNED:    minute
 ***********************************************************************/
uint16_t tiny32::TimeStamp_minute_encode(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mi) {
  uint16_t _numberofdays;
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    _numberofdays = days + 365 * y + (y + 3) / 4 - 1;    
    _numberofdays --;
    
    // Serial.printf("Debug: numberofminute = %d\r\n",(_numberofdays*24*60) + (h*60) + mi);
    return (_numberofdays*24*60) + (h*60) + mi;
}

/***********************************************************************
 * FUNCTION:    TimeStamp_24hr_encode
 * DESCRIPTION: Encoding time stamp
 * PARAMETERS:  h, mi (Example: 22,55)
 * RETURNED:    tempstamp
 ***********************************************************************/
uint16_t tiny32::TimeStamp_24hr_encode(uint16_t h, uint16_t mi)
{
    return ((h*60) + mi);
}

/***********************************************************************
 * FUNCTION:    TimeStamp_hour_minute_decode
 * DESCRIPTION: Decoding time stamp
 * PARAMETERS:  timestemp
 * RETURNED:    hour, minute
 ***********************************************************************/
void tiny32::TimeStamp_hour_minute_decode(uint16_t timestemp, uint16_t &h, uint16_t &mi)
{
    h=timestemp/60;
    mi=timestemp%60;
}

 /***********************************************************************
 * FUNCTION:    crc16_update
 * DESCRIPTION: CRC16 check
 * PARAMETERS:  uint16_t crc, uint8_t a
 * RETURNED:    uint16_t
 ***********************************************************************/
uint16_t tiny32::crc16_update(uint16_t crc, uint8_t a)
{
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
      if (crc & 1)
    crc = (crc >> 1) ^ 0xA001;
      else
    crc = (crc >> 1);
  }

  return crc;
}

/***********************************************************************
 * FUNCTION:    ec_modbusRTU
 * DESCRIPTION: EC sensor read
 * PARAMETERS:  address
 * RETURNED:    uS/cm
 ***********************************************************************/
uint16_t tiny32::ec_modbusRTU(uint8_t id)
{
  // #define modbusRTU_Debug
    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex = 0xffff;

    uint8_t _data_write[8];
    uint8_t _data_read[20];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[7];

      _data_write[0] = id;
      _data_write[1] = 0x03;
      _data_write[2] = 0x00;
      _data_write[3] = 0x01;
      _data_write[4] = 0x00;
      _data_write[5] = 0x01;

          // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

    
if(_byte_cnt == 7){
      _data_check[0] = _data_read[0];
      _data_check[1] = _data_read[1];
      _data_check[2] = _data_read[2];
      _data_check[3] = _data_read[3];
      _data_check[4] = _data_read[4];
      _data_check[5] = _data_read[5];
      _data_check[6] = _data_read[6]; 

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 7)
    {
      _data_check[0] = _data_read[_byte_cnt-7];
      _data_check[1] = _data_read[_byte_cnt-6];
      _data_check[2] = _data_read[_byte_cnt-5];
      _data_check[3] = _data_read[_byte_cnt-4];
      _data_check[4] = _data_read[_byte_cnt-3];
      _data_check[5] = _data_read[_byte_cnt-2];
      _data_check[6] = _data_read[_byte_cnt-1]; 

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    
   
    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    // Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {
      _temp_hex = _data_check[3];  //Serial.printf("Debug: _temp_hex = 0x%X\r\n",_temp_hex);
      _temp_hex = _temp_hex<<8;   //Serial.printf("Debug: _temp_hex = 0x%X\r\n",_temp_hex);
      _temp_hex = _temp_hex | _data_check[4]; //Serial.printf("Debug: _temp_hex = 0x%X(%d)\r\n",_temp_hex,_temp_hex);

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: Return DATA => %.d\r\n",_temp_hex);
      #endif

      return _temp_hex;
    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
      return 0xffff;
    }    
}


/***********************************************************************
 * FUNCTION:    ec_modbusRTU_begin
 * DESCRIPTION: set RX and TX pin
 * PARAMETERS:  rx, tx
 * RETURNED:    true/ false
 ***********************************************************************/
bool tiny32::ec_modbusRTU_begin(uint8_t rx, uint8_t tx)
{
  if( ((tx== TXD2) || (tx== TXD3)) && ((rx== RXD2) || (rx== RXD3)) )
  {
    rs485.begin(9600, SERIAL_8N1, rx, tx);
    return 1;
  }
  else
  {
    Serial.printf("Error: Fail to define RS485 port!!\r\n");
    return 0;
  }
  
}

/***********************************************************************
 * FUNCTION:    PZEM_016
 * DESCRIPTION: Read value from PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    true, false and reference parameter => volt, amp, power, energy, freq, pf
 ***********************************************************************/
bool tiny32::PZEM_016(uint8_t id, float &volt, float &amp, float &power, uint16_t &energy, float &freq, float &pf)
{
    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint32_t _temp_hex_32bit = 0x00000000;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[25];

      _data_write[0] = id;
      _data_write[1] = 0x04;
      _data_write[2] = 0x00;
      _data_write[3] = 0x00;
      _data_write[4] = 0x00;
      _data_write[5] = 0x0A;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 


       /**** correct data to buffer variable ****/
    if(_byte_cnt == 25){

    //Collect data
    for(int i=0; i<25; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 25){

      uint8_t _addcnt = _byte_cnt - 25; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<25; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return 0;
    }


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      //**** Read volt ****
      _temp_hex_16bit = _data_check[3];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[4]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      volt = (float)_temp_hex_16bit * 0.1;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: volt[hex] => %.d\r\n",_temp_hex_16bit);
      Serial.printf("Debug: volt[float] => %.1f\r\n",volt);
      #endif


      //**** Read Current ****
       _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[7];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[8]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[5];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[6]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      amp = (float)_temp_hex_32bit * 0.001;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: amp[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: amp[float] => %.1f\r\n",amp);
      #endif

      //**** Read Power ****
      _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[11];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[12]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[9];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[10]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      power = (float)_temp_hex_32bit * 0.1;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: power[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: power[float] => %.1f\r\n",power);
      #endif

      //**** Read Engergy ****
      _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[15];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[16]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[13];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[14]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      energy = (float)_temp_hex_32bit;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: energy[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: energy[float] => %.1f\r\n",energy);
      #endif

      //**** Read Frequency ****
      _temp_hex_16bit = _data_check[17];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[18]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      freq = (float)_temp_hex_16bit * 0.1;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: frequency[hex] => %.d\r\n",_temp_hex_16bit);
      Serial.printf("Debug: frequency[float] => %.1f\r\n",freq);
      #endif

      //**** Read power factor ****
      _temp_hex_16bit = _data_check[19];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[20]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      pf = (float)_temp_hex_16bit * 0.01;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: Power factor[hex] => %.d\r\n",_temp_hex_16bit);
      Serial.printf("Debug: Power factor[float] => %.01f\r\n",pf);
      #endif



       return 1;
    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return 0;
    }    
}


/***********************************************************************
 * FUNCTION:    PZEM_016_Volt
 * DESCRIPTION: Read value from PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    volt
 ***********************************************************************/
float  tiny32::PZEM_016_Volt(uint8_t id)
{
    float _volt;
    //#define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[25];

    _data_write[0] = id;
    _data_write[1] = 0x04;
    _data_write[2] = 0x00;
    _data_write[3] = 0x00;
    _data_write[4] = 0x00;
    _data_write[5] = 0x0A;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  



#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

    /**** correct data to buffer variable ****/
    if(_byte_cnt == 25){

    //Collect data
    for(int i=0; i<25; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 25){

      uint8_t _addcnt = _byte_cnt - 25; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<25; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return -1;
    }    

    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      //**** Read volt ****
      _temp_hex_16bit = _data_check[3];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[4]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      _volt = (float)_temp_hex_16bit * 0.1;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: volt[hex] => %.d\r\n",_temp_hex_16bit);
      Serial.printf("Debug: volt[float] => %.1f\r\n",_volt);
      #endif

      return _volt;

    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    } 
}


/***********************************************************************
 * FUNCTION:    PZEM_016_Amp
 * DESCRIPTION: Read value from PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    amp
 ***********************************************************************/
float  tiny32::PZEM_016_Amp(uint8_t id)
{
    float _amp;
    //#define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint32_t _temp_hex_32bit = 0x00000000;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[25];

    _data_write[0] = id;
    _data_write[1] = 0x04;
    _data_write[2] = 0x00;
    _data_write[3] = 0x00;
    _data_write[4] = 0x00;
    _data_write[5] = 0x0A;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  



#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

    /**** correct data to buffer variable ****/
    if(_byte_cnt == 25){

    //Collect data
    for(int i=0; i<25; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 25){

      uint8_t _addcnt = _byte_cnt - 25; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<25; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return -1;
    }

    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      //**** Read Current ****
       _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[7];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[8]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[5];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[6]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _amp = (float)_temp_hex_32bit * 0.001;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: amp[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: amp[float] => %.1f\r\n",_amp);
      #endif
      return _amp;

    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    } 
}


/***********************************************************************
 * FUNCTION:    PZEM_016_Power
 * DESCRIPTION: Read value from PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    power
 ***********************************************************************/
float  tiny32::PZEM_016_Power(uint8_t id)
{
   float _power;
    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint32_t _temp_hex_32bit = 0x00000000;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[25];

    _data_write[0] = id;
    _data_write[1] = 0x04;
    _data_write[2] = 0x00;
    _data_write[3] = 0x00;
    _data_write[4] = 0x00;
    _data_write[5] = 0x0A;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

        /**** correct data to buffer variable ****/
    if(_byte_cnt == 25){

    //Collect data
    for(int i=0; i<25; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 25){

      uint8_t _addcnt = _byte_cnt - 25; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<25; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return -1;
    }    



    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      //**** Read Power ****
      _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[11];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[12]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[9];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[10]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _power = (float)_temp_hex_32bit * 0.1;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: power[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: power[float] => %.1f\r\n",_power);
      #endif

      return _power;

    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    } 
}


/***********************************************************************
 * FUNCTION:    PZEM_016_Energy
 * DESCRIPTION: Read value from PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    energy
 ***********************************************************************/
int16_t tiny32::PZEM_016_Energy(uint8_t id)
{
 
   uint16_t _energy;
    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint32_t _temp_hex_32bit = 0x00000000;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[25];

    _data_write[0] = id;
    _data_write[1] = 0x04;
    _data_write[2] = 0x00;
    _data_write[3] = 0x00;
    _data_write[4] = 0x00;
    _data_write[5] = 0x0A;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

    /**** correct data to buffer variable ****/
    if(_byte_cnt == 25){

    //Collect data
    for(int i=0; i<25; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 25){

      uint8_t _addcnt = _byte_cnt - 25; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<25; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return -1;
    }    



    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      //**** Read Engergy ****
      _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[15];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[16]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[13];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[14]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _energy = (float)_temp_hex_32bit * 1;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: energy[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: energy[float] => %.1f\r\n",_energy);
      #endif

      return _energy;

    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    } 
}

/***********************************************************************
 * FUNCTION:    PZEM_016_Freq
 * DESCRIPTION: Read value from PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    freq
 ***********************************************************************/
float tiny32::PZEM_016_Freq(uint8_t id)
{
   float _freq;
    //#define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[25];

    _data_write[0] = id;
    _data_write[1] = 0x04;
    _data_write[2] = 0x00;
    _data_write[3] = 0x00;
    _data_write[4] = 0x00;
    _data_write[5] = 0x0A;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      



      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

    /**** correct data to buffer variable ****/
    if(_byte_cnt == 25){

    //Collect data
    for(int i=0; i<25; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 25){

      uint8_t _addcnt = _byte_cnt - 25; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<25; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return -1;
    }    


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      //**** Read Frequency ****
      _temp_hex_16bit = _data_check[17];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[18]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      _freq = (float)_temp_hex_16bit * 0.1;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: frequency[hex] => %.d\r\n",_temp_hex_16bit);
      Serial.printf("Debug: frequency[float] => %.1f\r\n",_freq);
      #endif

      return _freq;

    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    } 
}


/***********************************************************************
 * FUNCTION:    PZEM_016_PF
 * DESCRIPTION: Read value from PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    pf
 ***********************************************************************/
float tiny32::PZEM_016_PF(uint8_t id)
{
   float _pf;

    //#define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[25];

    _data_write[0] = id;
    _data_write[1] = 0x04;
    _data_write[2] = 0x00;
    _data_write[3] = 0x00;
    _data_write[4] = 0x00;
    _data_write[5] = 0x0A;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

    
    /**** correct data to buffer variable ****/
    if(_byte_cnt == 25){

    //Collect data
    for(int i=0; i<25; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 25){

      uint8_t _addcnt = _byte_cnt - 25; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<25; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return -1;
    }    


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      //**** Read power factor ****
      _temp_hex_16bit = _data_check[19];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[20]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      _pf = (float)_temp_hex_16bit * 0.01;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: Power factor[hex] => %.d\r\n",_temp_hex_16bit);
      Serial.printf("Debug: Power factor[float] => %.01f\r\n",_pf);
      #endif
      return _pf;


    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    } 
}


/***********************************************************************
 * FUNCTION:    PZEM_016_ResetEnergy
 * DESCRIPTION: Reset Energy value for PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    true, false
 ***********************************************************************/
bool tiny32::PZEM_016_ResetEnergy(uint8_t id) 
{
  // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;

    uint8_t _data_write[4];
    uint8_t _data_read[20];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[4];

    _data_write[0] = id;
    _data_write[1] = 0x42;


    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion



    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<4; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        Serial.printf("Debug: Byte count => %d\r\n",_byte_cnt);
        #endif
    } 

   
   if(_byte_cnt == 4){

     /*header data*/
      _data_check[0] = _data_read[0]; 
      _data_check[1] = _data_read[1]; 
      _data_check[2] = _data_read[2]; 
      _data_check[3] = _data_read[3];
  
      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
   if(_byte_cnt > 4){

    uint8_t _addcnt = _byte_cnt - 4;
    for(int i=0; i<4; i++)
    {
      _data_check[i] = _data_read[i+_addcnt]; 
    }

  
      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }    
    else
    {
      Serial.printf("Error: data error\r\n");
      return 0;
    }


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      Serial.printf("Info: PZEM-016 Reset Engergy Success\r\n");
      return 1;

    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return 0;
    } 

}

/***********************************************************************
 * FUNCTION:    PZEM_016_SetAddress
 * DESCRIPTION: Reset Energy value for PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  address id, new_id
 * RETURNED:   new_id
 ***********************************************************************/
int8_t tiny32::PZEM_016_SetAddress(uint8_t id, uint8_t new_id)
{

    /*check parameter*/
    if(new_id >= 0x7F)
    {
      Serial.printf("Error: Address is out of the range[1-127]\r\n");
      return -1;
    }

    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[8];

    _data_write[0] = id;
    _data_write[1] = 0x06;
    _data_write[2] = 0x00;
    _data_write[3] = 0x02;
    _data_write[4] = 0x00;
    _data_write[5] = new_id;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion




    /**** Write data ****/ 
    rs485.flush(); 
    for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485.available()>0);
      }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        Serial.printf("Byte Count = %d\r\n",_byte_cnt);
        #endif
    } 

    
   
   if(_byte_cnt == 8){

      for(int i=0; i<8; i++)
      {
        _data_check[i] = _data_read[i];
      }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 8){

      uint8_t _addcnt = _byte_cnt - 8;
      for(int i=0; i<8; i++)
      {
        _data_check[i] = _data_read[i+_addcnt]; 
      }



      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return -1;
    }


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      Serial.printf("Info: Success to set new Address[%d]\r\n",new_id);
      return new_id;
    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    } 
}


/***********************************************************************
 * FUNCTION:    PZEM_016_SearchAddress
 * DESCRIPTION: Search Address from PZEM-016 Modbus RTU power meter module
 * PARAMETERS:  nothing
 * RETURNED:    Address
 ***********************************************************************/
int8_t tiny32::PZEM_016_SearchAddress(void)
{

    uint8_t _id;

    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[25];

    /* Find ID from 1 to 127*/
    for(_id=1; _id<=127; _id++)
    {

        /*clear data*/
        _crc = 0xffff;
        _crc_r = 0xffff;
        _byte_cnt = 0;
        for(int i=0; i<sizeof(_data_check); i++)
        {
          _data_read[i]=0x00;
          _data_check[i]=0x00;
        }



        _data_write[0] = _id;
        _data_write[1] = 0x04;
        _data_write[2] = 0x00;
        _data_write[3] = 0x00;
        _data_write[4] = 0x00;
        _data_write[5] = 0x0A;

        // Generate CRC16
        for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
            _crc = crc16_update(_crc, _data_write[_i]);
        } 

      // Insert CRC16 to data byte
        #ifdef modbusRTU_Debug
        Serial.printf("_crc = 0x%02X\r\n",_crc);
        #endif
        _data_write[sizeof(_data_write)-1] = _crc >> 8;          
        _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  



    #pragma region 
        /***** Debug monitor ****/
        #ifdef modbusRTU_Debug
        Serial.printf("Data write(%d): [ ",sizeof(_data_write));
        for(byte _i=0; _i<sizeof(_data_write); _i++){
          if( _data_write[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_write[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_write[_i]);
          } 
        }
        Serial.printf("]\r\n");
        #endif
    #pragma endregion


        /**** Write data ****/ 
        rs485.flush(); 
        for(int _i=0; _i<8; _i++) rs485.write(_data_write[_i]);
        
        vTaskDelay(300);


        /**** Read data ****/
        if(rs485.available()){

        for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
        _byte_cnt = 0;

        //correct data
          do{
              _data_read[_byte_cnt++] = rs485.read();
              if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
                  _byte_cnt =0;
              }
          // }while(rs485.available()>0);
          }while(rs485.available()>0 && _byte_cnt<sizeof(_data_read));
          

          /***** Debug monitor ****/
          #ifdef modbusRTU_Debug
          Serial.printf("Data read(%d): [ ",_byte_cnt);
            for(byte _i=0; _i<_byte_cnt; _i++){
                if( _data_read[_i] > 0x0F ){
                  Serial.printf("0x%X ",_data_read[_i]);
                }
                else{
                  Serial.printf("0x0%X ",_data_read[_i]);
                } 
            }
            Serial.println("]");
            #endif
        } 

        
      
      if(_byte_cnt == 25){

          for(int i=0; i<25; i++)
          {
            _data_check[i] = _data_read[i];
          }

          /***** Debug monitor ****/
          #ifdef modbusRTU_Debug
          Serial.printf("Data check(%d): [ ",sizeof(_data_check));
          for(byte _i=0; _i<sizeof(_data_check); _i++){
              if( _data_check[_i] > 0x0F ){
                Serial.printf("0x%X ",_data_check[_i]);
              }
              else{
                Serial.printf("0x0%X ",_data_check[_i]);
              } 
          }
          Serial.println("]");
          #endif


           /*** crc check for data read ***/ 
        _crc = 0xffff;
        _crc_r = 0xffff;
        
        // Generate CRC16
        for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
            _crc = crc16_update(_crc, _data_check[_i]);
        } 
        #ifdef modbusRTU_Debug
        Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
        #endif

        // read crc byte from data_check
        _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
        _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
        _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
        #ifdef modbusRTU_Debug 
        Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
        #endif

        //return ON/OFF status
        if(_crc_r == _crc)
        {


          Serial.printf("\r\nInfo: the Address of this PZEM-016 => %d [Success]\r\n",_id);
          return _id;


        }  
        else
        {
          // Serial.printf("Error: crc16\r\n");
        } 


        
        
        }
      
      else if(_byte_cnt > 25)
      {
        
        uint8_t _addcnt = _byte_cnt - 25; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง
        //Collect data
        for(int i=0; i<25; i++)
        {
          _data_check[i] = _data_read[i+_addcnt];
        }
          /***** Debug monitor ****/
          #ifdef modbusRTU_Debug
          Serial.printf("Data check(%d): [ ",sizeof(_data_check));
          for(byte _i=0; _i<sizeof(_data_check); _i++){
              if( _data_check[_i] > 0x0F ){
                Serial.printf("0x%X ",_data_check[_i]);
              }
              else{
                Serial.printf("0x0%X ",_data_check[_i]);
              } 
          }
          Serial.println("]");
          #endif


           /*** crc check for data read ***/ 
        _crc = 0xffff;
        _crc_r = 0xffff;
        
        // Generate CRC16
        for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
            _crc = crc16_update(_crc, _data_check[_i]);
        } 
        #ifdef modbusRTU_Debug
        Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
        #endif

        // read crc byte from data_check
        _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
        _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
        _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
        #ifdef modbusRTU_Debug 
        Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
        #endif

        //return ON/OFF status
        if(_crc_r == _crc)
        {


          Serial.printf("\r\nInfo: the Address of this PZEM-016 => %d [Success]\r\n",_id);
          return _id;


        }  
        else
        {
          // Serial.printf("Error: crc16\r\n");
        } 

      }
      else
      {
        Serial.printf(".");

      }

    }

    Serial.printf("\r\nInfo: Finish searching .... Can't find PZEM-016 for this bus [fail]");
    return -1;

}



/***********************************************************************
 * FUNCTION:    PZEM_016_begin
 * DESCRIPTION: set RX and TX pin
 * PARAMETERS:  rx, tx
 * RETURNED:    true/ false
 ***********************************************************************/
bool tiny32::PZEM_016_begin(uint8_t rx, uint8_t tx)
{
  if( ((tx== TXD2) || (tx== TXD3)) && ((rx== RXD2) || (rx== RXD3)) )
  {
    rs485.begin(9600, SERIAL_8N1, rx, tx);
    return 1;
  }
  else
  {
    Serial.printf("Error: Fail to define RS485 port!!\r\n");
    return 0;
  }
  
}


/***********************************************************************
 * FUNCTION:    PZEM_003
 * DESCRIPTION: Read value from PZEM-003 Modbus RTU DC power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    true, false and reference parameter => volt, amp, power, energy
 ***********************************************************************/
bool tiny32::PZEM_003(uint8_t id, float &volt, float &amp, float &power, uint16_t &energy)
{
    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint32_t _temp_hex_32bit = 0x00000000;

    uint8_t _data_write[8];
    // uint8_t _data_read[21];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[21];

      _data_write[0] = id;
      _data_write[1] = 0x04;
      _data_write[2] = 0x00;
      _data_write[3] = 0x00;
      _data_write[4] = 0x00;
      _data_write[5] = 0x08;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485_2.flush(); 
    for(int _i=0; _i<8; _i++) rs485_2.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485_2.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485_2.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485_2.available()>0);
      }while(rs485_2.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        Serial.printf("Debug: Count byte => %d\r\n",_byte_cnt);
        #endif
    } 

    
    /**** correct data to buffer variable ****/
    if(_byte_cnt == 21){

    //Collect data
    for(int i=0; i<21; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 21){

      uint8_t _addcnt = _byte_cnt - 21; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<21; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return 0;
    }


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      //**** Read volt ****
      _temp_hex_16bit = _data_check[3];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[4]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      volt = (float)_temp_hex_16bit * 0.01;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: volt[hex] => %.d\r\n",_temp_hex_16bit);
      Serial.printf("Debug: volt[float] => %.1f\r\n",volt);
      #endif


      //**** Read current ****
      _temp_hex_16bit = _data_check[5];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[6]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      amp = (float)_temp_hex_16bit * 0.01;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: amp[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: amp[float] => %.1fA\r\n",amp);
      #endif


      //**** Read Power ****
       _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[9];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[10]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[7];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[8]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      power = (float)_temp_hex_32bit * 0.1;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: power[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: power[float] => %.1f\r\n",power);
      #endif

      //**** Read Engergy ****
      _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[13];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[14]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[11];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[12]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      energy = _temp_hex_32bit;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: energy[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: energy[float] => %d\r\n",energy);
      #endif



       return 1;
    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return 0;
    }    
}



/***********************************************************************
 * FUNCTION:    PZEM_003_Volt
 * DESCRIPTION: Read value from PZEM-003 Modbus RTU DC power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    volt
 ***********************************************************************/
float tiny32::PZEM_003_Volt(uint8_t id)
{
    float _volt;
    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint32_t _temp_hex_32bit = 0x00000000;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[21];

      _data_write[0] = id;
      _data_write[1] = 0x04;
      _data_write[2] = 0x00;
      _data_write[3] = 0x00;
      _data_write[4] = 0x00;
      _data_write[5] = 0x08;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485_2.flush(); 
    for(int _i=0; _i<8; _i++) rs485_2.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485_2.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485_2.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485_2.available()>0);
      }while(rs485_2.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

    
    /**** correct data to buffer variable ****/
    if(_byte_cnt == 21){

    //Collect data
    for(int i=0; i<21; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 21){

      uint8_t _addcnt = _byte_cnt - 21; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<21; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return 0;
    }

    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      //**** Read volt ****
      _temp_hex_16bit = _data_check[3];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[4]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      _volt = (float)_temp_hex_16bit * 0.01;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: volt[hex] => %.d\r\n",_temp_hex_16bit);
      Serial.printf("Debug: volt[float] => %.1f\r\n",_volt);
      #endif

      return _volt;

    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    }    
}


/***********************************************************************
 * FUNCTION:    PZEM_003_Amp
 * DESCRIPTION: Read value from PZEM-003 Modbus RTU DC power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    amp
 ***********************************************************************/
float tiny32::PZEM_003_Amp(uint8_t id)
{
    float _amp;
    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint32_t _temp_hex_32bit = 0x00000000;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[21];

      _data_write[0] = id;
      _data_write[1] = 0x04;
      _data_write[2] = 0x00;
      _data_write[3] = 0x00;
      _data_write[4] = 0x00;
      _data_write[5] = 0x08;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485_2.flush(); 
    for(int _i=0; _i<8; _i++) rs485_2.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485_2.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485_2.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485_2.available()>0);
      }while(rs485_2.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

    /**** correct data to buffer variable ****/
    if(_byte_cnt == 21){

    //Collect data
    for(int i=0; i<21; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 21){

      uint8_t _addcnt = _byte_cnt - 21; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<21; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return 0;
    }

    

    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {


      //**** Read current ****
      _temp_hex_16bit = _data_check[5];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[6]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);
      _amp = (float)_temp_hex_16bit * 0.01;

      #ifdef modbusRTU_Debug
      Serial.printf("Debug: amp[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: amp[float] => %.1fA\r\n",_amp);
      #endif

      return _amp;

    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    }    
}



/***********************************************************************
 * FUNCTION:    PZEM_003_Power
 * DESCRIPTION: Read value from PZEM-003 Modbus RTU DC power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    power
 ***********************************************************************/
float tiny32::PZEM_003_Power(uint8_t id)
{
    float _power;
    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint32_t _temp_hex_32bit = 0x00000000;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[21];

      _data_write[0] = id;
      _data_write[1] = 0x04;
      _data_write[2] = 0x00;
      _data_write[3] = 0x00;
      _data_write[4] = 0x00;
      _data_write[5] = 0x08;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485_2.flush(); 
    for(int _i=0; _i<8; _i++) rs485_2.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485_2.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485_2.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485_2.available()>0);
      }while(rs485_2.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

        /**** correct data to buffer variable ****/
    if(_byte_cnt == 21){

    //Collect data
    for(int i=0; i<21; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 21){

      uint8_t _addcnt = _byte_cnt - 21; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<21; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return 0;
    }


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

     


      //**** Read Power ****
       _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[9];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[10]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[7];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[8]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _power = (float)_temp_hex_32bit * 0.1;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: power[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: power[float] => %.1f\r\n",_power);
      #endif

      return _power;

     
    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    }    
}


/***********************************************************************
 * FUNCTION:    PZEM_003_Energy
 * DESCRIPTION: Read value from PZEM-003 Modbus RTU DC power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    energy
 ***********************************************************************/
int16_t tiny32::PZEM_003_Energy(uint8_t id)
{
    uint16_t _energy;
    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint16_t _temp_hex_16bit = 0xffff;
    uint32_t _temp_hex_32bit = 0x00000000;

    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[21];

      _data_write[0] = id;
      _data_write[1] = 0x04;
      _data_write[2] = 0x00;
      _data_write[3] = 0x00;
      _data_write[4] = 0x00;
      _data_write[5] = 0x08;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion


    /**** Write data ****/ 
    rs485_2.flush(); 
    for(int _i=0; _i<8; _i++) rs485_2.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485_2.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485_2.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485_2.available()>0);
      }while(rs485_2.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        #endif
    } 

    /**** correct data to buffer variable ****/
    if(_byte_cnt == 21){

    //Collect data
    for(int i=0; i<21; i++)
    {
      _data_check[i] = _data_read[i];
    }


      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else if(_byte_cnt > 21){

      uint8_t _addcnt = _byte_cnt - 21; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<21; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    }
    else
    {
      Serial.printf("Error: data error\r\n");
      return 0;
    }


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

    
      //**** Read Engergy ****
      _temp_hex_32bit = 0x00000000; //clear buffer

      _temp_hex_16bit = _data_check[13];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8;   //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[14]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _temp_hex_32bit = _temp_hex_32bit<<16; //Serial.printf("Debug: _temp_hex_32bit = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);

      _temp_hex_16bit = _data_check[11];  //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit<<8; //Serial.printf("Debug: _temp_hex_16bit = 0x%X\r\n",_temp_hex_16bit);
      _temp_hex_16bit = _temp_hex_16bit | _data_check[12]; //Serial.printf("Debug: _temp_hex_16bit = 0x%X(%d)\r\n",_temp_hex_16bit,_temp_hex_16bit);

      _temp_hex_32bit = _temp_hex_32bit | _temp_hex_16bit; //Serial.printf("Debug: _temp_hex_32bit* = 0x%X(%d)\r\n",_temp_hex_32bit,_temp_hex_32bit);
      _energy = _temp_hex_32bit;
      
      #ifdef modbusRTU_Debug
      Serial.printf("Debug: energy[hex] => %.d\r\n",_temp_hex_32bit);
      Serial.printf("Debug: _energy[float] => %d\r\n",_energy);
      #endif

      return _energy;
    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    }    
}




/***********************************************************************
 * FUNCTION:    PZEM_003_ResetEnergy
 * DESCRIPTION: Reset Energy value for PZEM-003 Modbus RTU power meter module
 * PARAMETERS:  address(id)
 * RETURNED:    true, false
 ***********************************************************************/
bool tiny32::PZEM_003_ResetEnergy(uint8_t id) 
{
    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;

    uint8_t _data_write[4];
    uint8_t _data_read[20];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[4];

    _data_write[0] = id;
    _data_write[1] = 0x42;


    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion



    /**** Write data ****/ 
    rs485_2.flush(); 
    for(int _i=0; _i<4; _i++) rs485_2.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485_2.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485_2.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485_2.available()>0);
      }while(rs485_2.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        Serial.printf("Debug: Byte Count => %d\r\n",_byte_cnt);
        #endif
    } 

   
  if(_byte_cnt == 4){

    /*header data*/
    _data_check[0] = _data_read[0]; 
    _data_check[1] = _data_read[1]; 
    _data_check[2] = _data_read[2]; 
    _data_check[3] = _data_read[3];

    for(int i=0; i<_byte_cnt; i++)
    {
      _data_check[i] = _data_read[i]; 
    }

    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data check(%d): [ ",sizeof(_data_check));
    for(byte _i=0; _i<sizeof(_data_check); _i++){
        if( _data_check[_i] > 0x0F ){
          Serial.printf("0x%X ",_data_check[_i]);
        }
        else{
          Serial.printf("0x0%X ",_data_check[_i]);
        } 
    }
    Serial.println("]");
    #endif

  }
  else if( _byte_cnt > 4)
  {
    uint8_t _addcnt = _byte_cnt - 4;
    for(int i=0; i<4; i++)
    {
      _data_check[i] = _data_read[i+_addcnt]; 
    }

    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data check(%d): [ ",sizeof(_data_check));
    for(byte _i=0; _i<sizeof(_data_check); _i++){
        if( _data_check[_i] > 0x0F ){
          Serial.printf("0x%X ",_data_check[_i]);
        }
        else{
          Serial.printf("0x0%X ",_data_check[_i]);
        } 
    }
    Serial.println("]");
    #endif

  } 
  else
  {
    Serial.printf("Error: data error\r\n");
    return 0;
  }


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      
      Serial.printf("Info: PZEM-003 Reset Engergy Success\r\n");
      return 1;

    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return 0;
    } 

}

/***********************************************************************
 * FUNCTION:    PZEM_003_SetAddress
 * DESCRIPTION: Reset Energy value for PZEM-003 Modbus RTU power meter module
 * PARAMETERS:  address id, new_id
 * RETURNED:   new_id
 ***********************************************************************/
int8_t tiny32::PZEM_003_SetAddress(uint8_t id, uint8_t new_id)
{

    /*check parameter*/
    if(new_id >= 0x7F)
    {
      Serial.printf("Error: Address is out of the range[1-127]\r\n");
      return -1;
    }

    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[8];

    _data_write[0] = id;
    _data_write[1] = 0x06;
    _data_write[2] = 0x00;
    _data_write[3] = 0x02;
    _data_write[4] = 0x00;
    _data_write[5] = new_id;

    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
        _crc = crc16_update(_crc, _data_write[_i]);
    } 

  // Insert CRC16 to data byte
    #ifdef modbusRTU_Debug
    Serial.printf("_crc = 0x%02X\r\n",_crc);
    #endif
    _data_write[sizeof(_data_write)-1] = _crc >> 8;          
    _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  


#pragma region 
    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data write(%d): [ ",sizeof(_data_write));
    for(byte _i=0; _i<sizeof(_data_write); _i++){
      if( _data_write[_i] > 0x0F ){
        Serial.printf("0x%X ",_data_write[_i]);
      }
      else{
        Serial.printf("0x0%X ",_data_write[_i]);
      } 
    }
    Serial.printf("]\r\n");
    #endif
#pragma endregion




    /**** Write data ****/ 
    rs485_2.flush(); 
    for(int _i=0; _i<8; _i++) rs485_2.write(_data_write[_i]);
    
    vTaskDelay(300);


    /**** Read data ****/
    if(rs485_2.available()){

    for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
    _byte_cnt = 0;

    //correct data
      do{
          _data_read[_byte_cnt++] = rs485_2.read();
          if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
              _byte_cnt =0;
          }
      // }while(rs485_2.available()>0);
      }while(rs485_2.available()>0 && _byte_cnt<sizeof(_data_read));
      

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data read(%d): [ ",_byte_cnt);
        for(byte _i=0; _i<_byte_cnt; _i++){
            if( _data_read[_i] > 0x0F ){
              Serial.printf("0x%X ",_data_read[_i]);
            }
            else{
              Serial.printf("0x0%X ",_data_read[_i]);
            } 
        }
        Serial.println("]");
        Serial.printf("Byte count = %d\r\n",_byte_cnt);
        #endif
    } 

    
   
  if(_byte_cnt == 8){

 
      _data_check[0] = _data_read[0];
      _data_check[1] = _data_read[1];
      _data_check[2] = _data_read[2]; 

      _data_check[3] = _data_read[3];
      _data_check[4] = _data_read[4];

      _data_check[5] = _data_read[5];
      _data_check[6] = _data_read[6]; 
      _data_check[7] = _data_read[7];
     

      /***** Debug monitor ****/
      #ifdef modbusRTU_Debug
      Serial.printf("Data check(%d): [ ",sizeof(_data_check));
      for(byte _i=0; _i<sizeof(_data_check); _i++){
          if( _data_check[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_check[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_check[_i]);
          } 
      }
      Serial.println("]");
      #endif
    } 
  else if( _byte_cnt > 8)
  {
    uint8_t _addcnt = _byte_cnt - 8;
    for(int i=0; i<8; i++)
    {
      _data_check[i] = _data_read[i+_addcnt]; 
    }

    /***** Debug monitor ****/
    #ifdef modbusRTU_Debug
    Serial.printf("Data check(%d): [ ",sizeof(_data_check));
    for(byte _i=0; _i<sizeof(_data_check); _i++){
        if( _data_check[_i] > 0x0F ){
          Serial.printf("0x%X ",_data_check[_i]);
        }
        else{
          Serial.printf("0x0%X ",_data_check[_i]);
        } 
    }
    Serial.println("]");
    #endif
    
  }   
  else
    {
      Serial.printf("Error: data error\r\n");
      return -1;
    }


    /*** crc check for data read ***/ 
    _crc = 0xffff;
    _crc_r = 0xffff;
    
    // Generate CRC16
    for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
        _crc = crc16_update(_crc, _data_check[_i]);
    } 
    #ifdef modbusRTU_Debug
    Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
    #endif

    // read crc byte from data_check
    _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
    _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
    #ifdef modbusRTU_Debug 
    Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
    #endif

    //return ON/OFF status
    if(_crc_r == _crc)
    {

      Serial.printf("Info: Success to set new Address[%d]\r\n",new_id);
      return new_id;
    }  
    else
    {
      Serial.printf("Error: crc16\r\n");
       return -1;
    } 
}


/***********************************************************************
 * FUNCTION:    PZEM_003_SearchAddress
 * DESCRIPTION: Search Address from PZEM-003 Modbus RTU power meter module
 * PARAMETERS:  nothing
 * RETURNED:    Address
 ***********************************************************************/

int8_t tiny32::PZEM_003_SearchAddress(void)
{

    uint8_t _id;

    // #define modbusRTU_Debug

    uint16_t _crc = 0xffff;
    uint16_t _crc_r = 0xffff;
    uint8_t _data_write[8];
    uint8_t _data_read[40];
    uint8_t _byte_cnt = 0;
    uint8_t _data_check[21];

    /* Find ID from 1 to 127*/
    for(_id=1; _id<=127; _id++)
    {

        /*clear data*/
        _crc = 0xffff;
        _crc_r = 0xffff;
        _byte_cnt = 0;
        for(int i=0; i<sizeof(_data_check); i++)
        {
          _data_read[i]=0x00;
          _data_check[i]=0x00;
        }


      _data_write[0] = _id;
      _data_write[1] = 0x04;
      _data_write[2] = 0x00;
      _data_write[3] = 0x00;
      _data_write[4] = 0x00;
      _data_write[5] = 0x08;

        // Generate CRC16
        for(byte _i=0; _i < sizeof(_data_write)-2; _i++){
            _crc = crc16_update(_crc, _data_write[_i]);
        } 

      // Insert CRC16 to data byte
        #ifdef modbusRTU_Debug
        Serial.printf("_crc = 0x%02X\r\n",_crc);
        #endif
        _data_write[sizeof(_data_write)-1] = _crc >> 8;          
        _data_write[sizeof(_data_write)-2]= _crc - _data_write[sizeof(_data_write)-1]*0x0100 ;  



    #pragma region 
        /***** Debug monitor ****/
        #ifdef modbusRTU_Debug
        Serial.printf("Data write(%d): [ ",sizeof(_data_write));
        for(byte _i=0; _i<sizeof(_data_write); _i++){
          if( _data_write[_i] > 0x0F ){
            Serial.printf("0x%X ",_data_write[_i]);
          }
          else{
            Serial.printf("0x0%X ",_data_write[_i]);
          } 
        }
        Serial.printf("]\r\n");
        #endif
    #pragma endregion


        /**** Write data ****/ 
        rs485_2.flush(); 
        for(int _i=0; _i<8; _i++) rs485_2.write(_data_write[_i]);
        
        vTaskDelay(300);


        /**** Read data ****/
        if(rs485_2.available()){

        for(byte _i=0; _i<sizeof(_data_read); _i++) _data_read[_i] = 0x00; //clear buffer
        _byte_cnt = 0;

        //correct data
          do{
              _data_read[_byte_cnt++] = rs485_2.read();
              if(_data_read[0] == 0x00){ //แก้ไช bug เนื่องจากอ่านค่าแรกได้ 0x00
                  _byte_cnt =0;
              }
          // }while(rs485_2.available()>0);
          }while(rs485_2.available()>0 && _byte_cnt<sizeof(_data_read));
          

          /***** Debug monitor ****/
          #ifdef modbusRTU_Debug
          Serial.printf("Data read(%d): [ ",_byte_cnt);
            for(byte _i=0; _i<_byte_cnt; _i++){
                if( _data_read[_i] > 0x0F ){
                  Serial.printf("0x%X ",_data_read[_i]);
                }
                else{
                  Serial.printf("0x0%X ",_data_read[_i]);
                } 
            }
            Serial.println("]");
            #endif
        } 

      /* Collect data to variable buffer */
      if(_byte_cnt == 21){

        for(int i=0; i<21; i++)
        {
          _data_check[i] = _data_read[i];
        }
     

            /***** Debug monitor ****/
            #ifdef modbusRTU_Debug
            Serial.printf("Data check(%d): [ ",sizeof(_data_check));
            for(byte _i=0; _i<sizeof(_data_check); _i++){
                if( _data_check[_i] > 0x0F ){
                  Serial.printf("0x%X ",_data_check[_i]);
                }
                else{
                  Serial.printf("0x0%X ",_data_check[_i]);
                } 
            }
            Serial.println("]");
            #endif


            /*** crc check for data read ***/ 
          _crc = 0xffff;
          _crc_r = 0xffff;
          
          // Generate CRC16
          for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
              _crc = crc16_update(_crc, _data_check[_i]);
          } 
          #ifdef modbusRTU_Debug
          Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
          #endif

          // read crc byte from data_check
          _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
          _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
          _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
          #ifdef modbusRTU_Debug 
          Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
          #endif

          //return ON/OFF status
          if(_crc_r == _crc)
          {


            Serial.printf("\r\nInfo: the Address of this PZEM-003 => %d [Success]\r\n",_id);
            return _id;


          }  
          else
          {
            // Serial.printf("Error: crc16\r\n");
            return -1;
          }
      } 

      else if(_byte_cnt > 21){

      uint8_t _addcnt = _byte_cnt - 21; //ตัวแปรชดเชยการอ่านค่าผิดตำแหน่ง

      //Collect data
      for(int i=0; i<21; i++)
      {
        _data_check[i] = _data_read[i+_addcnt];
      }

            /***** Debug monitor ****/
            #ifdef modbusRTU_Debug
            Serial.printf("Data check(%d): [ ",sizeof(_data_check));
            for(byte _i=0; _i<sizeof(_data_check); _i++){
                if( _data_check[_i] > 0x0F ){
                  Serial.printf("0x%X ",_data_check[_i]);
                }
                else{
                  Serial.printf("0x0%X ",_data_check[_i]);
                } 
            }
            Serial.println("]");
            #endif


            /*** crc check for data read ***/ 
          _crc = 0xffff;
          _crc_r = 0xffff;
          
          // Generate CRC16
          for(byte _i=0; _i < sizeof(_data_check)-2; _i++){
              _crc = crc16_update(_crc, _data_check[_i]);
          } 
          #ifdef modbusRTU_Debug
          Serial.printf("Debug: _crc = 0x%X\r\n",_crc);
          #endif

          // read crc byte from data_check
          _crc_r = _data_check[sizeof(_data_check)-1]; //Serial.print(">>"); Serial.println(_crc_r,HEX);
          _crc_r = _crc_r <<8; //Serial.print(">>"); Serial.println(_crc_r,HEX);
          _crc_r = _crc_r + _data_check[sizeof(_data_check)-2]; //Serial.print(">>"); Serial.println(_crc_r,HEX);      
          #ifdef modbusRTU_Debug 
          Serial.printf("Debug: _crc_r = 0x%X\r\n",_crc_r);
          #endif

          //return ON/OFF status
          if(_crc_r == _crc)
          {


            Serial.printf("\r\nInfo: the Address of this PZEM-003 => %d [Success]\r\n",_id);
            return _id;


          }  
          else
          {
            // Serial.printf("Error: crc16\r\n");
            return -1;
          }     


      }
      else
      {
        Serial.printf(".");

      }

    }

    Serial.printf("\r\nInfo: Finish searching .... Can't find PZEM-003 for this bus [fail]");

}


/***********************************************************************
 * FUNCTION:    PZEM_003_begin
 * DESCRIPTION: set RX and TX pin
 * PARAMETERS:  rx, tx
 * RETURNED:    true/ false
 ***********************************************************************/
bool tiny32::PZEM_003_begin(uint8_t rx, uint8_t tx)
{
  if( ((tx== TXD2) || (tx== TXD3)) && ((rx== RXD2) || (rx== RXD3)) )
  {
    rs485_2.begin(9600, SERIAL_8N2, rx, tx);
    return 1;
  }
  else
  {
    Serial.printf("Error: Fail to define RS485 port!!\r\n");
    return 0;
  }
  
}