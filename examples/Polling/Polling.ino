/*****************************************************
 * This UartEvent example shows the simple use of
 * polling to receive data using Serial1 port. While
 * this example does not do anything special but use
 * the traditional polling methods.
 *****************************************************/
#include <UartEvent.h>

Uart1Event pollEvent;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    //--------------------------pollEvent Configuration-----------------------------------
    pollEvent.loopBack = true;// internal loopback set / "default = false"
    pollEvent.begin(9600);
    //------------------------------------------------------------------------------------
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        pollEvent.print(c);
    }
    
    if(pollEvent.available()) {
        char c = pollEvent.read();
        Serial.print(c);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
