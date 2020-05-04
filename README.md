[//]: # (file: "~/Privat/R3P/room_tech/README.md")
# RoomTech
This software is controlling 16 motors. 8 are used to open/close windows and 8 are used to raise and lower the window blinds. The neutral position is a closed not blinded window.


## Hardware

### Window opener/closer and blind raise/lower motors
Each window has two AC motors whith three cables, the neutral conductor (N) and two phase conductors (P0,P1), one for each direction to drive. 

The P0 phase conductor is used to close the window or raise the blind. The P1 phase conductor is used to open the window or lower the blind. P0 and P1 may not be given simultaniously!

### Relays
Two relays are used to control a motor. The first relay switches the current on and off. The second relay switches the current to P0 or P1. The neutral position of the relays are: Relay 0 no voltage to the motor, relay 1 voltage switched to P0. The following table shows the combinations of the relays to control the motor:

<table border="1">
  <tr><th>relay 0</th><th>relay 1</th><th> P0 </th><th> P1 </th><th>action</th></tr>
  <tr><td> off   </td><td> off   </td><td> 0V </td><td> 0V </td><td>stop</td></tr>
  <tr><td> off   </td><td>  on   </td><td> 0V </td><td> 0V </td><td>stop</td></tr>
  <tr><td>  on   </td><td> off   </td><td>230V</td><td> 0V </td><td>close/raise</td></tr>
  <tr><td>  on   </td><td>  on   </td><td>0V  </td><td>230V</td><td>open/lower</td></tr>
</table>

The relay boards are powerd by 12V and controlled by 5 volt IO pins. The IO connector offers GND, 5V VCC and IO input pins.

The GND pins of all relay boards, shift register logic may be connected and Raspberry pi can be connected.
The 5V VCC of one relay board can be used to power the shift register logic and the Raspberry pi.

It has to be checked if the 5V VCC pins of all relay boards may be connected. (CHECK !!!)

Source: https://www.az-delivery.de/products/16-relais-modul?_pos=2&_sid=c065e0f4f&_ss=r

[Jakob]

### Shift Register as relay controler 
8 relays are controlled by a 8 bit shift register (74HC595). So each shift register can control 2 windows, each with 2 on/off relays and 2 direction relays:

<table border="1">
  <tr><th>bit</th><th>function</th></tr>
  <tr><td>0  </td><td>window 1, opener/closer on/off</td></tr>
  <tr><td>1  </td><td>window 1, opener/closer open/close</td></tr>
  <tr><td>2  </td><td>window 1, blind on/off</td></tr>
  <tr><td>3  </td><td>window 1, blind lower/raise</td></tr>
  <tr><td>4  </td><td>window 2, opener/closer on/off</td></tr>
  <tr><td>5  </td><td>window 2, opener/closer open/close</td></tr>
  <tr><td>6  </td><td>window 2, blind on/off</td></tr>
  <tr><td>7  </td><td>window 2, blind lower/raise</td></tr>
</table>

The shift register is controled by 3 output data pins (DS,ST_CP,SH_CP) of a Rasperry Pi GPIO. The DS pin is the new data to shift into the register.
When the SH_CP (Serial shift clock) pin level is rising, serial data input register will do a shift.
The ST_CP pin is the parallel update output. The rising edge updates the data output by the shift register state.

The shift registers can be powered by 2~6 volt. For this project we use the 5V VCC pin of a relay board IO connector.

[Jakob]


### Microcomputer as controler and UserInterface

A Raspberry Pi microcomputer is used to control the motor controller relay shift registers by the GPIO 

[Jakob]


### Light dimmer hardware

First idea: Arduino PWM controlled [dimmer1](https://www.instructables.com/id/AC-PWM-Dimmer-for-Arduino/) or [dimmer2](http://www.inmojo.com/store/inmojo-market/item/digital-ac-dimmer-module-lite-v.2/).

Alternate Hardware: [sonoff](https://sonoff.tech/product/wifi-diy-smart-switches/4ch-r2-pro-r2).

Hardware Explained: [video1](https://www.youtube.com/watch?v=XdCvJ1wZ0bA) and [here](https://electronics.stackexchange.com/questions/339185/modify-pwm-controlled-230v-incandescent-bulb-dimmer-circuit).

But the lights may have a fallback dimmer.

[Jakob]

Arduino Software and serial interface to the UI [Eduard]


### Tablet as UserInterface device

A tablet running a firefox browser in kiosk mode.

[Holger]



## Software

The software is a motor position controler running the full time. Offering a https interface. And a web application. 


### Position controler

The position controler offers a set of commands to request and set the motor positions by https requests

- Motor parameter are stored in a configuration file.
- There is no motor position encoder, so the controler has to determine the position by the motor speed, runtime and direction.
- To determine the current position on initialization the controler can read it from a file or it does a initialization run by driving the motor to an end position.
- Threaded Python Application
  - Webserver => The Webserver thread reads and writes into the Motor controler by mutex protected methods
	- Session handling and login
    - File requests
	- Controler requests
	- Pushed Events
  - Motor controler => The Motor controler
  - Timer events => The Timer events callbacks the motor controler if a position is reached.
  

### Web Application

#### Design [Holger]


#### HTML realization [Eduard]



## Time line and responsibilities

- 2020-02-?? Project meeting
- 2020-02-20 Git Hub project configuration [Eduard]
- 2020-03-31 Rough Web design [Holger]
- 2020-03-31 Tablet kiosk mode website access [Holger]
- 2020-03-31 Shift register board and connections to relay boards and Raspberry Pi [Jakob]
- 2020-03-31 Raspberry pi motor controler [Eduard]


