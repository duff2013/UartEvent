/*****************************************************
 * This SerialEvent example shows the simple use of
 * polling to receive data using Serial1 port. While 
 * this example does not do anything special but use 
 * the traditional polling methods. Notice no user 
 * buffer is declared so a internal 2 byte buffer will 
 * be used instead.
 *****************************************************/
#include <SerialEvent.h>

SerialEvent pollEvent = SerialEvent();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  /****************************************************
   * The following is the minmum that must be defined 
   * for this SerialEvent library to work.
   ****************************************************/
//--------------------------------------------------------------------------------------
  pollEvent.port = &Serial1; // set port to Serial1, this must be called before "begin"
  pollEvent.begin(9600);// start SerialEvent using Hardware Serial1
//--------------------------------------------------------------------------------------
}

void loop() {
  if(pollEvent.available()) {
    char c = pollEvent.read();
    Serial.print(c);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

