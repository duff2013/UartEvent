SerialEvent
=========

<h3>Teensy 3.0/3.1 SerialEvent Library</h3>

<b>Overview:</b><br>
>SerialEvents use DMA for transfering and Recieving data in the "background". 
Events that are supported currently are:<br>
1.  Recieve Buffer Full<br>
2.  Read Bytes Until<br>

<b>Transmitting:</b><br>
> While tradional sending of serial data is byte by byte based this uses a packet based sending. By using the Teensy's DMA engine SerialEvent allows the user to send data and move on very quickly, thus freeing up CPU overhead. Since the this class's base is Print most of the normal print statements work as you would expect.<br>
Care must be taken because currently the library uses dynamic memory to send data. While small data transfers are safe for the most part large data transfers can use up considerable ram, up to 5520 bytes, so budgiting your memory is critical.<br>

<b>Recieving:</b><br>
> Users have more control over how they want to capture data using this library. 
<br>

The performance is on par with the Hardware Serial Class in the Teensduino core<br>
but is a little slower in actual sending because of the DMA setup overhead. I'm<br>
working on speeding this up. While the actual sending of the data is slower the<br>
process of sending data is much faster in that it only takes microseconds to get<br>
through the print statement no matter what baud rate. Sending a 4000 byte packet<br>
takes about ~1200 microseconds compared to ~82000 microseconds with the normal<br>
Uart print method at 115200. This is because the FIFO is a packet based buffer<br>
instead of a byte based. Mulitple packets can be stored with 5520 bytes total<br>
maximum currently.<br>
<br>
Events that are currently
While this code is still in beta it works pretty well but much more testing is<br>
happening.<br>
</ul>
