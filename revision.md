><b>Updated (4/13/15)</b><br>
  * Refactored code for more efficient use of the serial DMA. 
  * Now functions 'available' and 'read' work interchangeably in the rx event or in user code. This update was due to a bug where reading data from the rx event and polling you had to clear the rx buffer. Now this is not necessary. 
  * Buffer full will not now cause an event to trigger, this will be re added in the future. 
  * Event callbacks are now called from a low priority isr so it can be preempted by the dma isr. 
  * The write function now will truncate packet to the available buffer space at that time if greater than what is available.

><b>Updated (3/24/15)</b><br>
  * Fix for RX buffer size of 1</h5>

><b>Updated</b><br>
  * to the formerly SerialEvent library. I changed the name to not be confused with Teensyduino serialEvent. 
  * Lots of internal changes where made to how the DMA is configuring the internal RXTX buffers and most importantly how the size of these buffers are changed. No longer can you declare the size of the buffer in the sketch it has been moved to the UartEvent.h file. This is do to alignment of the buffer was doubling the reported ram usage. While it was not noticeable for small buffer sizes, once it got to about 1024 bytes it would start to explode. 
  * Also these buffer sizes have to be a power of 2.