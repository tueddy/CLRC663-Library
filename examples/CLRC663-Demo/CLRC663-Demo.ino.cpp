/*
  The MIT License (MIT)

  Copyright (c) 2023 tueddy (Dirk Carstensen) 

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "CLRC663.h"


/*

  This example shows how to use the library on an Arduino-compatible platform, in this case the testing has been done
  on a ESP32.

  In the setup() function, there are some custom register settings which are you probably have to uncomment or change
  such that they are in the correct configuration for your hardware.
*/


// Pin to select the hardware, the CS pin.
#define CHIP_SELECT 5

// IRQ pin for wake-up from LPCD
#define IRQ_PIN 16



// Pins MOSI, MISO and SCK are connected to the default pins, and are manipulated through the SPI object.
// By default that means MOSI=23, MISO=19, SCK=18. (N)SS: 5

// SPI instance
CLRC663 reader(&SPI, CHIP_SELECT, IRQ_PIN);
// I2C instance (address 0x28-0x2B)
//CLRC663 reader(0x2A, IRQ_PIN);



// Hex print for blocks without printf.
void print_block(uint8_t * block,uint8_t length){
    for (uint8_t i=0; i<length; i++){
        if (block[i] < 16){
          Serial.print("0");
          Serial.print(block[i], HEX);
        } else {
          Serial.print(block[i], HEX);
        }
        Serial.print(" ");
    }
    Serial.println("");
}



void setup(){
  // Start serial communication.
  Serial.begin(115200);
  Serial.println("startup CLRC633 RFID-reader..");
//  reader.begin(SDA, SCL);
  reader.begin();
  
  // get version
  Serial.print("CLRC663 version: ");
  Serial.println(reader.getVersion());
}


void loop(){
  uint8_t uid[10]={0};  //variable for 10byte UID
  bool cardFound = false;
  Serial.println("loop..");
  reader.softReset();          
  // Set the registers of the CLRC633 into the default for ISO-14443
  reader.AN1102_recommended_registers(MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER);
  uint8_t uid_len = reader.read_iso14443_uid(uid);

  if (uid_len != 0) {  // did we get an UID?
      Serial.print("ISO-14443 tag found! UID of ");
      Serial.print(uid_len);
      Serial.print(" bytes: ");
      print_block(uid, uid_len);
      Serial.print("\n");
      delay(100);
      return;
  }

  reader.softReset();  
  reader.AN1102_recommended_registers(MFRC630_PROTO_ISO15693_1_OF_4_SSC); 

  uint8_t password[] = {0x0F, 0x0F, 0x0F, 0x0F};
  uid_len = reader.read_iso18693_uid(uid, password);
  if (uid_len != 0) { 
      Serial.print("ISO-18693 tag found! UID of ");
      Serial.print(uid_len);
      Serial.print(" bytes: ");
      print_block(uid, uid_len);
      Serial.print("\n");
      delay(100);
      return;
  }
  // no card found!
  //

  // measure LPCD
  uint8_t iValue;
  uint8_t qValue;
  reader.AN11145_start_IQ_measurement(&iValue, &qValue);
  Serial.println("IQ_measurement:");
  Serial.print("in-phase (I): ");
  Serial.println(iValue);
  Serial.print("quadrature (Q): ");
  Serial.println(qValue);

 
  // go to LPCD mode
  Serial.println("goto sleep..");
  reader.softReset();       
  reader.lpcd_start(iValue, qValue);

  // wait for LPCD wakeup-IRQ 
  while (digitalRead(IRQ_PIN) == LOW) {
    delay(100);
  }
  Serial.println("wakeup!");
  uint8_t irq1 = reader.get_irq1();
  if ((irq1 && MFRC630_IRQ1_LPCD_IRQ) > 0)
    Serial.println("LPCD IRQ!");
  uint8_t i_result = reader.read_reg(MFRC630_REG_LPCD_I_RESULT);
  uint8_t q_result = reader.read_reg(MFRC630_REG_LPCD_Q_RESULT);

  Serial.print("in-phase (I): ");
  Serial.println(i_result);
  Serial.print("quadrature (Q): ");
  Serial.println(q_result);
  delay(1000);
}



