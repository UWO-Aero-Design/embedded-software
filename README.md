

<!-- <p align="center">
  <img width="200" height="200" src="logo.png">
</p> -->

# Western Aero Design Team - Embedded Software

## Goal

The goal of this project is to create an onboard system for Western Aero's advanced class plane for competing at SAE Aero Design East.
At a bare minimum, it should be able to:
- Relay it's current altitude to the ground station
- Actuate servos via ground station command for payload drops

Ideally the entire system should seamless integrate the mechanical design of the plane with the ground station to allow for easy operator use. The design should allow for easy testing of individual components and hot swapping of peripherals, if necessary. 


## Running

Make sure you have the [Arduino IDE](https://www.arduino.cc/en/main/software) installed.

Change any defined pinouts to reflect your schematic setup.

Upload and run the code, using the dip switches to select system modes:
|Mode|Dip Pattern  |
|--|--|
|Full Test System|0000|
|Test TX Serial|0001|
|ZTR Demo #1|0010|
|Pitot Demo|0011|
|Enviro Demo|0100|
|IMU Demo|0101|
|GPS Demo|0110|
|Comp System|1111|


#### Notes

 - This project is designed to be run on Western Aero Design's [Onboard-Systems PCB](https://github.com/UWO-Aero-Design/circuit-design).
