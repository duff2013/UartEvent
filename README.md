UartEvent
=========
<h3>Teensy 3.1 UartEvent Library V6.1</h3>

<h5>Update -> (3/24/15) Fix for RX buffer size of 1</h5>

<h5>Update to the formerly SerialEvent library. I changed the name to not be confused with Teensyduino serialEvent. Lots of internal changes where made to how the DMA is configuring the internal RXTX buffers and most importantly how the size of these buffers are changed. No longer can you declare the size of the buffer in the sketch it has been moved to the UartEvent.h file. This is do to alignment of the buffer was doubling the reported ram usage. While it was not noticeable for small buffer sizes, once it got to about 1024 bytes it would start to explode. Also these buffer sizes have to be a power of 2.</h5>


<h5>This library is only intended for Teensy 3.1 Hardware Serial since it has enough DMA channels for all 3 serial ports. While you can use it with the Teensy 3.0 the limited memory and DMA channels make it more suited for the Teensy3.1. This is not for use with the USB Serial.</h5>

UartEvents use DMA for transferring and Receiving data in the "background" making a full-duplex serial communications more possible. By using the DMA to handle the serial i/o you can lesson the load on the Teensy while sending and receiving. This library intends to hand over more control to the user by using Events which give you more flexibility.
Events that are supported currently are:<br>
1.  Receive Buffer Full<br>
2.  Read Bytes Until<br>
3.  1 Byte Receive<br>
4.  Transmit Complete<br>

<b>Memory:</b><br>
> This library allows you to declare how much memory for transmitting and receiving data. Rx and TX memory is just an uint8_t array buffer. RX memory is can be any size while TX memory has to be a power of two.<br>

<b>Transmitting:</b><br>
> While traditional sending of serial data is byte by byte based this uses a packet based sending. By using the Teensy's DMA engine, UartEvent allows the user to send data and move on very quickly, thus freeing up CPU overhead. Since the base class is "Stream" most of the normal print statements work as you would expect. A couple of caveats need to be expressed now.<br>
The reason the library use this setup, it allows you to just copy the data to its buffers and move on, not having to wait for each character to send. This makes it somewhat none blocking. The biggest overhead in sending is only just coping data to the transmit buffer.<br>

<b>Receiving:</b><br>
> Users have more control over how they want to capture data using this library using events instead of polling methods. While you can use polling, registering events can be quite useful. The three events that are currently implemented are explained below:<br>
1.  Receive Buffer Full: This will call an event handler function when the user defined buffer is full.<br>
2.  Read Bytes Until: This will call an event handler function when a termination character is detected.<br>
3.  1 Byte Receive: This will call an event handler function when a byte is present.<br><br>
The DMA engine allows you to receive data which will signal an event handler that is just a function that you define in the main sketch.
<br>

<b>Performance:</b><br>
>The performance is on par with the Hardware Serial Class in the Teensyduino core, with the biggest advantage is being able to send a data and move on very quickly. Sending a 5000 byte packet takes only ~260uS @24MHz to get throught the print statement code while this would take much longer using the standard Serial print..<br>
</ul>

<b>Usage:</b><br>
>Since your event functions are called from interrupts make sure you declare any variables used in the event handler as volatile! Also make sure any code inside a event is fast as possible since it is called from the DMA ISR.
