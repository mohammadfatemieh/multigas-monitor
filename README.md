# VAMoS MultiGas Monitor
Research instrument for monitoring gaseous concentrations.

## Overview
The VAMoS MultiGas Monitor is an improvement to the exisiting MGM used by SFU Volcanology. The current MGM cannot be used for more than
1-2 hours and must be manually controlled by an on-site researcher. VAMoS is a device that can be deployed for multi-week remote
monitoring in the glacial environment on Mt. Meager's Job Glacier.

## Features
* Onboard microprocessing using the ESP32 micrcontroller
* Inverted-F trace antenna for WiFi transmission at 2.4GHz
* 4-20mA signal processing for state-of-the-art Alphasense electrochemical and NDIR gas sensors
* Onboard BME280 temperature, humidity, and pressure sensor; XA1100 GPS module
* Connector for external DS18B20 temperature probe
* microSD card connector for backup data storage

## Known Issues
* footprint for FT231XS is too large
* footprint for coin cell battery holder is incorrect
* footprint for J2 missing silkscreen
* holes for hook type test points too small
* linear regulators heating to 90 C at 500mA operation

## Features for Revision 0.01
* LED power indicator for different power rails
* LED indicators to show system health and operation status
* DC-DC converters prior to linear regulators
* Solar charging circuit
* Terminal block interface for easy sensor replacement

## Schematic Changes
* Ensure all sheets are same size and have the same orientation
* Create logocom schematic header
* Flip all upside down text 
* Remove redundant net labels if port labels already exist
* Add capacitor voltage ratings 
* Remove any text and move symbols so that nothing overlaps anything else 
* Break up Amplifier.schdoc so that it can fit on landscape page
* New TP symbols
* Make TP designators visible
* New ferrite bead symbol 
* Change designator naming schematic to 100... 200... 300...
* U3 'SHIELD' text??
* Fill in top sheet
* Remove redundant 3.3V supplies
* New crystal/oscillator symbols