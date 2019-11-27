# TinyLCC
#### code authored by: Sherjeel Shehzad

This repository contains code to be deployed on an ATMEGA8 microcontroller that implements the control system for a linear motor compressor, intended for use in a refrigerator or similar appliance. Features include:
* A complete system operating parameter detection suite, including (but not limited to):
  * RMS Operating Voltage
  * Operating Frequency
  * RMS Current draw
  * Mean average Power draw
* Transmission of all parameters in a semi-JSON-compliant format over UART
* Stall Detection
* (Software-based) short-circuit protection
* Motor resonant frequency detection based on motor back-emf voltage readings
