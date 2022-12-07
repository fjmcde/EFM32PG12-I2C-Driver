#  Silicon Labs EFM32 I²C driver
I developed a driver for the I²C protocol on the Silicon Labs EFM32 Pearl Gecko development board. This was an assignment for my ECEN 2370 Embedded Software Engineering class. In this lab we learned about the I²C protocol, how to develop an interrupt driven state machine, implement an application scheduler, and how to implement proper design techniques in order to keep application code separate from driver code. This is built off of my Lab5 I2C repository, as the two assignments had different requirements.

# Features
• Compatible with either the I2C0 or I2C1 peripheral on the EFM32 Peark Gecko.\
• Interfaces with the onboard Si7021 and external SHTC3 Temperature and Humidity sensors.\
• Capable of measuring the relative humidity and temperature of the surrounding environment.\
• Can handle 8-bit and 16-bit data transmission (read or write).\

# Working on ...
• Handling Checksum (CRC).\
• Adding functionality for enumerated commands that currently aren't implemented.\
