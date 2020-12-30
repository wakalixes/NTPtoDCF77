# NTPtoDCF77

The purpose of this project is to enable syncronizing an automatic clock to a NTP server instead of to the DCF77 radio signal. 
The hardware emulates the well-known decoded DCF77 signal, such that the DCF77 syncronization of the clock can be reused. It does not create or send a DCF77 radio signal.

## Hardware

This project uses the [Joy-IT ESP32 module](https://joy-it.net/de/products/SBC-NodeMCU-ESP32) and level shifter to drive circuits with higher supply voltage than 3.3 V.
The level shifter is made by a simple NPN transistor with a 1k resistor on its base, which essentially creates an open-collector output.

