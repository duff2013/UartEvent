/*****************************************************
 * This SerialEvent example shows the use of the 
 * loopback feature that the Teensy has for testing 
 * purposes and also how to send and receive large sets 
 * of data using TX and RX events. Since we are using 
 * the DMA to send and receive you will see that there 
 * is very little CPU overhead.
 *
 * Transmitting data is packet based so even sending
 * large array of data takes much less time in the write
 * function than would be using the normal Serial1 write.
 *
 * Receiving allows the user to declare his/her own
 * buffer and size. If you don't know the size of your
 * receiving data you can use either polling mode or
 * set your buffer size to 1.
 *****************************************************/
 
#include <SerialEvent.h>
#include "monroe.h"

#define TX_BUFFER_SIZE 6000
#define RX_BUFFER_SIZE 128

Serial1Event Event1 = Serial1Event();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(0);
  while(!Serial);
  //---------------------------------------------------------------------------------------
  // Must declare TX buffer size in bytes, this must be as big as your largest packet
  SERIAL1_MEMORY_TX(TX_BUFFER_SIZE);
  // Must declare RX buffer size in bytes, will fire when buffer is full or term if declared
  SERIAL1_MEMORY_RX(RX_BUFFER_SIZE);
  Event1.loopBack = true;// internal loopback set / "default = false"
  Event1.txEventHandler = tx1Event;// event handler Serial1 TX
  Event1.rxEventHandler = rx1Event;// event handler Serial1 RX
  Event1.rxTermCharacter = '\n'; // RX termination character
  Event1.begin(9600);// start Serial1
  //---------------------------------------------------------------------------------------
}

void loop() {
  // write a data packet
  Event1.write(monroe, 5510);
  // wait for data to flush
  Event1.flush();
}

//--------------------------------------Serial1 Events----------------------------------
void tx1Event(void) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  // TX Event function will fired when the DMA is finished sending a packet
  //Serial.print("\ntx1 Event Fired\n");
}

void rx1Event(void) {
  // RX Event function prints the buffer when it is full
  // Now "Event1.rxBuffer" is used to read the buffer, no user buffer needed
  Serial.printf("%s", Event1.rxBuffer);

}
