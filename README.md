# Arduino Uno Tripcomputer for the KW1282 KKL K-Line protocol

## Welcome
Simple trip computer for the Arduino Uno with a 16x2 Screen to display useful information in all VAG vehicles with the MARELLI 4LV ECU (VAG Number: 036906034AM). This includes most Golf mk4/Jetta/Bora from the years around 2000 that are limited to the K-Line communication and the KW1281 protocol. Newer cars since around 2005 began to adapt OBD-2 with CAN communication, that can be accessed through a ELM327 controller. No such simple controller exists currently for KKL.

This repo contains all necessary files. Buy an Arduino Uno, a 16x2 Screen to stick on the board and the Autodia K409 KKL OBD to USB cable (or similar).

## Setup
Find the RX and TX connections on the AutoDia K409 board (open the OBD-site plastic) and cut them where appropiate. Refer to the [linked git repo](https://github.com/mkirbst/lupo-gti-tripcomputer-kw1281) for pictures and a bit more info. Hook the working side of both connections where you just cut to the RX and TX pins of your Arduino Uno (Watch out, since RX(Recieve) and TX(Transmit) depends on the perspective) by cutting the USB Male plug.  

## Credit
Thanks to many wonderful projects for making this project less painful than it already is.

[Blafusel](https://www.blafusel.de/obd/obd2_kw1281.html) with a detailed overview on the KW1281 protocol and the communication between the microcontroller and the ECU.

[mkirbst's existing code](https://github.com/mkirbst/lupo-gti-tripcomputer-kw1281) helped a lot to get the  mainframe going. His code did not work on my car, although he has a very similar one to mine using the same protocol. He refers to some connection problems, which I also got that can only be caused by the software. This project eliminates connection problems by implementing a procedure for error messages by the ECU. 

## Caution
This is an early version and I'm only releasing it to help on anyones journey with this VAG mess. You need to manually remove connections and solder cables on an OBD to USB board and hook them up to the Arduino Uno TX and RX pins. You need to turn ignition ON for the ECU to start. This software should not break anything in the ECU, since only the measure groups are accessed. Depending on your car, you may need to adapt this code for various values. 

This project may work with other Arduino's, Displays, OBD cables and VAG cars.

## Problems
The OBD 12V from the car should be enough to power on the Arduino, but in my case the +12V and Ground lines of the USB cable do nothing, so just hook it up like a smartphone through the cigarrete lighter USB.

If you have questions feel free to send me a message. Will accept all merge requests if they work. 


