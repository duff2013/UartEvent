><b>Updated (9/14/15 v6.6)</b><br>
* Thanks to Teensy forum user doughboy -> https://forum.pjrc.com/members/38767-doughboy for finding these issues.
* Now return the number of bytes sucessfully sent since the driver can truncate user data if available buffer is to small.
* Fixed Fill_Internal_Buffer_Time.ino for when compiled without optimizations and using printf for floating point values.
* Fixed many warnings.

><b>Updated (7/23/15 v6.5)</b><br>
* Again thanks to Teensy forum user Barney -> https://forum.pjrc.com/members/36714-Barney for finding these issues.
* Fixed baud rate generation for Uart2 and Uart3, they where using Uart1 "Baud2Div".

><b>Updated (7/14/15 v6.4)</b><br>
* Thanks to Teensy forum user Barney -> https://forum.pjrc.com/members/36714-Barney for finding these issues.
* Fixed rx buffer full, it was triggering every received byte before now it triggers when the buffer is actually full.
* Fixed "Serial3" not working because the internal buffer was set which "Serial3" does not have.
* Fixed all serial ports end function, reattaching wrong interrupt vector.

><b>Updated (4/13/15 v6.3)</b><br>
  * New trigger for incoming data "rxBufferSizeTrigger", you can set a trigger for how of the rx buffer is used.
  * Renamed rxTermCharacter to rxTermCharacterTrigger.
  * Users RX/TX callbacks now is called from low priority ISR's'.

><b>Updated (4/13/15)</b><br>
  * Refactored code for more efficient use of the serial DMA. 
  * Now functions 'available' and 'read' work interchangeably in the rx event or in user code. This update was due to a bug where reading data from the rx event and polling you had to clear the rx buffer. Now this is not necessary. 
  * Buffer full will not now cause an event to trigger, this will be re added in the future. 
  * Event callbacks are now called from a low priority isr so it can be preempted by the dma isr. 
  * The write function now will truncate packet to the available buffer space at that time if greater than what is available.

><b>Updated (3/24/15)</b><br>
  * Fix for RX buffer size of 1

><b>Updated</b><br>
  * to the formerly SerialEvent library. I changed the name to not be confused with Teensyduino serialEvent. 
  * Lots of internal changes where made to how the DMA is configuring the internal RXTX buffers and most importantly how the size of these buffers are changed. No longer can you declare the size of the buffer in the sketch it has been moved to the UartEvent.h file. This is do to alignment of the buffer was doubling the reported ram usage. While it was not noticeable for small buffer sizes, once it got to about 1024 bytes it would start to explode. 
  * Also these buffer sizes have to be a power of 2.