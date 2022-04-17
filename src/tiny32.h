/***********************************************************************
 * File         :     tiny32.h
 * Description  :     Class for Hardware config and function for blynkide32_v2 module
 * Author       :     Tenergy Innovation Co., Ltd.
 * Date         :     23 Nov 2021
 * Revision     :     1.5
 * Rev1.0       :     Original 
 * Rev1.1       :     Add TimeStamp_minute  
 *                    Add TimeStamp_24hr_minute
 * Rev1.2             Add EC RS485 sensor    
 * Rev1.3             Change define switch to int  
 * Rev1.4       :     Add EC sensor (RS485)    
 * Rev1.5       :     Add PZEM-016 Energy Meter AC    
 *                    Add PZEM-003 Energy Meter DC  
 *                    Add rs485_2.begin(9600, SERIAL_8N2, RXD2, TXD2) for PZEM-003    
 * website      :     http://www.tenergyinnovation.co.th
 * Email        :     uten.boonliam@innovation.co.th
 * TEL          :     089-140-7205
 ***********************************************************************/

#ifndef TINY32_H
#define TINY32_H
#include "Ticker.h"


class tiny32
{
private:
#define version_c  "1.5"

    /* data */

public:
/**************************************/
/*           GPIO define              */
/**************************************/
#define RXD2    16
#define TXD2    17
#define RXD3    27
#define TXD3    26 
#define SW1     33
#define SW2     14
#define RELAY   25
#define LED_D5  4
#define LED_D1  12
#define SLID_SW  15
#define BUZZER  13

tiny32(/* args */);
void Relay(bool state);
void RedLED(bool state);
void BlueLED(bool state);
void BuildinLED(bool state);
void buzzer_beep(int times);
bool Sw1(void);
bool Sw2(void);
bool Slid_sw(void);
void class_version(void);

private:
uint8_t _resolution_bit;
uint16_t crc16_update(uint16_t crc, uint8_t a);


public:
void TickBlueLED(float second);
void TickRedLED(float second);
void TickBuildinLED(float second);
bool PWM_Setup(uint8_t channel, double freq, uint8_t resolution_bit, uint8_t pin);
bool PWM_Drive(uint8_t channel, uint8_t percentage);
uint16_t TimeStamp_minute_encode(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mi);
uint16_t TimeStamp_24hr_encode(uint16_t h, uint16_t mi);
void TimeStamp_hour_minute_decode(uint16_t timestemp, uint16_t &h, uint16_t &mi);

private:
uint16_t ec_modbusRTU(uint8_t id);

public:
/* PZEM-016 Modbus RTU AC power meter module */
bool   PZEM_016(uint8_t id, float &volt, float &amp, float &power, uint16_t &engergy, float &freq, float &pf);
float  PZEM_016_Volt(uint8_t id);
float  PZEM_016_Amp(uint8_t id);
float  PZEM_016_Power(uint8_t id);
int16_t  PZEM_016_Energy(uint8_t id);
float  PZEM_016_Freq(uint8_t id);  
float  PZEM_016_PF(uint8_t id);
bool PZEM_016_ResetEnergy(uint8_t id);
int8_t PZEM_016_SetAddress(uint8_t id, uint8_t new_id);
int8_t PZEM_016_SearchAddress(void);


/* PZEM-003 Modbus RTU DC power meter module */
bool   PZEM_003(uint8_t id, float &volt, float &amp, float &power, uint16_t &engergy);
float  PZEM_003_Volt(uint8_t id);
float  PZEM_003_Amp(uint8_t id);
float  PZEM_003_Power(uint8_t id);
int16_t  PZEM_003_Energy(uint8_t id);
bool PZEM_003_ResetEnergy(uint8_t id);
int8_t PZEM_003_SetAddress(uint8_t id, uint8_t new_id);
int8_t PZEM_003_SearchAddress(void);

};

#endif