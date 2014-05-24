SerialEvent
=========

<h3>Teensy 3.0/3.1 SerialEvent Library</h3>

SerialEvents use DMA for transferring and Receiving data in the "background" making a full-duplex serial communications more possible. By using the DMA to handle the serial i/o you can lesson the load on the Teensy while sending and receiving. This library intends to hand over more control to the user by using Events which give you more flexibility.
Events that are supported currently are:<br>
1.  Receive Buffer Full<br>
2.  Read Bytes Until<br>
3.  1 Byte Receive<br>
4.  Transmit Complete<br>

<b>Transmitting:</b><br>
> While traditional sending of serial data is byte by byte based this uses a packet based sending. By using the Teensy's DMA engine, SerialEvent allows the user to send data and move on very quickly, thus freeing up CPU overhead. Since the base class is "Print" most of the normal print statements work as you would expect.<br><br>
Care must be taken because currently the library uses dynamic memory to send data. While small data transfers are safe for the most part large data transfers can use up considerable ram, up to 5520 bytes, so budgeting your memory is critical.<br>

<b>Receiving:</b><br>
> Users have more control over how they want to capture data using this library using events instead of polling methods. While you can use polling, registering events can be quite useful. The three events that are currently implemented are explained below:<br>
1.  Receive Buffer Full: This will call an event handler function when the user defined buffer is full.<br>
2.  Read Bytes Until: This will call an event handler function when a termination character is detected.<br>
3.  1 Byte Receive: This will call an event handler function when a byte is present.<br><br>
The DMA engine allows you to receive data which will signal an event handler that is just a function that you define in the main sketch.
<br>

<b>Performance:</b><br>
>The performance is on par with the Hardware Serial Class in the Teensduino core but is a little slower in actual sending because of the DMA setup overhead. I'm working on speeding this up. While the actual sending of the data is slower the process of sending data is much faster in that it only takes microseconds to get through the print statement no matter what baud rate. Sending a 4000 byte packet takes about ~1200 microseconds compared to ~82000 microseconds with the normal Uart print method at 115200. This is because the FIFO is a packet based buffer instead of a byte based. Multiple packets can be stored with 5520 bytes total maximum currently.<br>
</ul>

<b>Usage:</b><br>
>Since your event functions are called from interrupts make sure you declare any variables as volatile!