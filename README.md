#  Silicon Labs EFM32 I²C driver
I developed a driver for the I²C protocol on the Silicon Labs EFM32 Pearl Gecko development board. This was an assignment for my ECEN 2370 Embedded Software Engineering class. In this lab we learned about the I²C protocol, how to develop an interrupt driven state machine, implement an application scheduler, and how to implement proper design techniques in order to keep application code separate from driver code. This is built off of my Lab5 I2C repository, as the two assignments had different requirements.

# Features
• Compatible with either the I2C0 or I2C1 peripheral on the EFM32 Peark Gecko.
• Interfaces with the onboard Si7021 (in the application layer).
• Capable of measuring the relative humidity of the surrounding environment.

# Working on ...
• Adding functionality to read temperature data from the relative humidity reading.
• Adding functionality to write to user specific registers on the Si7021.
• Refactoring state machine code to include additional states.