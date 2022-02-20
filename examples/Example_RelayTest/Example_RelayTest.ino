/***********************************************************************
 * Project      :     Example_RelayTest
 * Description  :     Test Relay on tiny32 board
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

void setup() {
  Serial.begin(115200);
  Serial.printf("\r\n**** Example_RelayTest ****\r\n");
}

void loop() {
  
  /**** one beep ****/
  Serial.printf("Relay => ON\r\n");
  mcu.Relay(0); //NO disconnect COM  
  vTaskDelay(1000);

  /**** two beep ****/
  Serial.printf("Relay => OFF\r\n");
  mcu.Relay(1); //NO connect COM
  vTaskDelay(1000);

  
}
