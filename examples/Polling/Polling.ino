/*****************************************************
 * This SerialEvent example shows the simple use of
 * polling to receive data using Serial1 port. While 
 * this example does not do anything special but use 
 * the traditional polling methods.
 *****************************************************/
#include <SerialEvent.h>

Serial1Event pollEvent;

#define TX_BUFFER_SIZE 64
#define RX_BUFFER_SIZE 64

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  /****************************************************
   * The following is the minmum that must be defined
   * for this SerialEvent library to work.
   ****************************************************/
  // Must declare TX buffer size in bytes, this must be as big as your largest packet
  SERIAL1_MEMORY_TX(TX_BUFFER_SIZE);
  // Must declare RX buffer size in bytes, will fire when buffer is full or term if declared
  SERIAL1_MEMORY_RX(RX_BUFFER_SIZE);
  // start SerialEvent using Hardware Serial1
  pollEvent.begin(9600);
}

void loop() {
  if(pollEvent.available()) {
    char c = pollEvent.read();
    Serial.print(c);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}