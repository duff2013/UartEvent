/*****************************************************
 * SerialEvent uses DMA for RX, TX which reduces CPU
 * load.
 *
 * This SerialEvent example shows the simple use of
 * polling to receive data using Serial1 port. While 
 * this example does not do anything special but use 
 * the traditional polling methods. Notice no user 
 * buffer is declared so a internal 2 byte buffer will 
 * be used instead.
 *****************************************************/
#include <SerialEvent.h>

SerialEvent Event1 = SerialEvent();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //---------------------------------------------------------------------------------------------
  /* The following must be defined for this Serial DMA library to work */
  Event1.loopBack = false; // internal loopback set / "default = false"
  Event1.port = &Serial1; // set port to Serial1
  Event1.begin(9600);// start Serial1 DMA
  //---------------------------------------------------------------------------------------------
}

void loop() {
  if(Event1.available()) {
    char c = Event1.read();
    Serial.print(c);
  }
}

