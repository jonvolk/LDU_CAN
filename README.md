# LDU_CAN

Mapping of inverter parameters over CAN

Must use brake pressure input on POT2!
Parameter index values based on SINE Firmware 4.90.R!

Openinvertercustom CAN mapping. 

rpm,309,0,16,1,TX
pot,275,0,16,1,TX
pot2,275,16,16,TX
din_brake,79,8,1,TX

All other parameters are set via CAN SDO standard.

