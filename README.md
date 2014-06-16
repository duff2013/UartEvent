SerialEvent
=========

<h3>Teensy 3.0/3.1 SerialEvent Library</h3>

SerialEvents use DMA for transferring and Receiving data in the "background" making a full-duplex serial communications more possible. By using the DMA to handle the serial i/o you can lesson the load on the Teensy while sending and receiving. This library intends to hand over more control to the user by using Events which give you more flexibility.
Events that are supported currently are:<br>
1.  Receive Buffer Full<br>
2.  Read Bytes Until<br>
3.  1 Byte Receive<br>
4.  Transmit Complete<br>

<b>Memory:</b><br>
> This library allows you to declare how much memory for transmitting and receiving data. Rx memory is just an array buffer while transmitting uses a fifo of buffers. Each TX fifo buffer is 128 bytes so when declaring TX memory the fifo size is: (TX MEMORY/128) + 1.<br><br>

<b>Transmitting:</b><br>
> While traditional sending of serial data is byte by byte based this uses a packet based sending. By using the Teensy's DMA engine, SerialEvent allows the user to send data and move on very quickly, thus freeing up CPU overhead. Since the base class is "Stream" most of the normal print statements work as you would expect. A couple of caveats need to be expressed now.<br><br>
This library uses a fifo buffers which are hard coded at 128 bytes each. The fifo size explained in the Memory section effects sending multiple print statments in a row. Since each print statement will take up a fifo if it less than 128 bytes and more if it is larger some prints might get lost because of the fifo size limitation. It is recommended to stuff as much data into one print statement as possible. The reason the library use this setup, it allows you to just copy the data to the fifo buffers and move on, not having to wait for each character to send. This makes it somewhat none blocking. The biggest overhead in sending is only just coping data to the fifo. <br><br>

<b>Receiving:</b><br>
> Users have more control over how they want to capture data using this library using events instead of polling methods. While you can use polling, registering events can be quite useful. The three events that are currently implemented are explained below:<br>
1.  Receive Buffer Full: This will call an event handler function when the user defined buffer is full.<br>
2.  Read Bytes Until: This will call an event handler function when a termination character is detected.<br>
3.  1 Byte Receive: This will call an event handler function when a byte is present.<br><br>
The DMA engine allows you to receive data which will signal an event handler that is just a function that you define in the main sketch.
<br>

<b>Performance:</b><br>
>The performance is on par with the Hardware Serial Class in the Teensduino core but is a little slower in actual sending because of the coping the data to the fifo and DMA setup overhead. I'm working on speeding this up. While the actual sending of the data is slower the process of sending data is much faster in that it only takes microseconds to get through the print statement no matter what baud rate.<br><br>
</ul>

<b>Usage:</b><br>
>Since your event functions are called from interrupts make sure you declare any variables used in the event handler as volatile! Also make sure any code inside a event is fast as possible since it is called from the DMA ISR.
