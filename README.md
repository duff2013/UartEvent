UartEvent
=========
<h3>Teensy 3.1 UartEvent Library v6.6</h3>

<h5>This library is only intended for Teensy 3.1 Hardware Serial, (no USB) since it has enough DMA channels for all 3 serial ports. While you can use it with the Teensy 3.0 the limited memory and DMA channels make it more suited for the Teensy3.1. This is not for use with the USB Serial. Teensy LC is not supported.</h5>

Events that are supported currently are:<br>
1.  Receive Buffer Size<br>
2.  Receive Bytes Until<br>
3.  1 Byte Receive<br>
4.  Transmit Complete<br>

<b>Transmitting:</b><br>
> While traditional sending of serial data is byte by byte based this uses a packet based sending. By using the Teensy's DMA engine, UartEvent allows the user to send data and move on very quickly, thus freeing up CPU overhead. Since the base class is "Stream" most of the normal print statements work as you would expect. A couple of caveats need to be expressed now.<br>
The reason the library use this setup, it allows you to just copy the data to its buffers and move on, not having to wait for each character to send. This makes it non-blocking. The biggest overhead in sending is only just coping data to the transmit buffer which has been optimized for speed.<br>

<b>Receiving:</b><br>
> Users have more control over how they want to capture data using this library using events instead of polling methods. While you can use polling, registering events can be quite useful. The three events that are currently implemented are explained below:<br>
1.  Receive Buffer Size: This will call an event handler function when the user defined buffer is size is reached.<br>
2.  Read Bytes Until: This will call an event handler function when a termination character is detected.<br>
3.  1 Byte Receive: This will call an event handler function when a byte is present.<br><br>
The DMA engine allows you to receive data which will signal an event handler that is just a function that you define in the main sketch.
<br>

<b>Performance:</b><br>
>The performance is on par with the Hardware Serial Class in the Teensyduino core, with the biggest advantage is being able to send a data and move on very quickly. Sending a 5000 byte packet takes only ~260uS @24MHz to get through the print statement code while this would take much longer using the standard Serial print..<br>
</ul>

<b>Usage:</b><br>
>Since your event functions are called from interrupts make sure you declare any variables used in the event handler as volatile! Also make sure any code inside a event is fast as possible since it is called from the an ISR. DO NOT use any delay functions!
