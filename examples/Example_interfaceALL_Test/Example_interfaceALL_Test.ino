/***********************************************************************
 * Project      :     Example_interfaceALL_Test
 * Description  :     Test switch, LED, Relay, Buzzer .. ea
 * Hardware     :     tiny32         
 * Author       :     Tenergy Innovation Co., Ltd.
 * Date         :     19/02/2022
 * Revision     :     1.0
 * website      :     http://www.tenergyinnovation.co.th
 * Email        :     admin@innovation.co.th
 * TEL          :     +66 82-380-3299
 ***********************************************************************/
#include <tiny32.h>

tiny32 mcu; //define object
HardwareSerial RS485(1);

unsigned long interval = 0;


void setup() {
  Serial.begin(115200);
  RS485.begin(9600, SERIAL_8N1, 16, 17);
  Serial.printf("\r\n**** Example_interfaceALL_Test ****\r\n");
  RS485.printf("\r\n**** Example_interfaceALL_Test ****\r\n");
  mcu.buzzer_beep(1);
}

void loop() {
  
  static int _state = 0; 

  /* check SW1 */
  if(mcu.Sw1())
  {
    mcu.buzzer_beep(1);
    Serial.println("SW1 is pressing[serial]\r\n");
    RS485.println("SW1 is pressing[rs485]\r\n");
    while(mcu.Sw1());
    _state++;
    if(_state > 4) _state =0;
  }
  /* check SW2 */
  else if(mcu.Sw2())
  {
    mcu.buzzer_beep(3);
    Serial.println("SW2 is pressing[serial]\r\n");
    RS485.println("SW2 is pressing[rs485]\r\n");
    while(mcu.Sw2());
    _state = 0;
  }

  /* check value of _state variable */
  switch (_state)
  {
    case 1:
      mcu.RedLED(1);
      break;

    case 2:
      mcu.RedLED(0);
      mcu.BlueLED(1);
      break; 

    case 3:
      mcu.BlueLED(0);
      mcu.BuildinLED(1);
      break;    

    case 4:
      mcu.BuildinLED(0);
      mcu.Relay(1);
      break;  

    case 0:
      mcu.RedLED(0);
      mcu.BlueLED(0);
      mcu.BuildinLED(0);
      mcu.Relay(0);
      break;
  }

  /* check status of slid switch every 5 second */
  if(millis() - interval > 5000)
  {
    Serial.printf("SLID SWITCH => %d\r\n",mcu.Slid_sw());
    RS485.printf("SLID SWITCH => %d\r\n",mcu.Slid_sw());
    interval = millis();
  }

  vTaskDelay(300);

}
