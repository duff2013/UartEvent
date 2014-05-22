SerialEvent
=========

<h3>Teensy 3.0/3.1 SerialEvent Library</h3>

<b>Overview:</b><br>
>SerialEvents use DMA for transfering data in the "background".
Events that are supported currently are: 
1. Recieve Buffer Full<br>
2. Read Bytes Until<br>

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
