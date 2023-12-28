Github CI build provides binaries for separate hardware platforms:

OBC - EMx Version (ClimbObc@TKN_RELEASE@-obc.axf)
---------------------------------------
This version runs on dedicated Climb OBC hardware.
<p align="left" width="100%">
    <img  width="20%" src="pic/obc.jpg"> 
</p>


DevBoard - BA_OM13085 Version (ClimbObc@TKN_RELEASE@-dev.axf)
---------------------------------------
This version runs on the developer board where some PIO pinnings are modified.
<p align="left" width="100%">
    <img  width="20%" src="pic/OM13085.jpg"> 
</p>


	Hardware needs fololowing external connections:
	

	J2(J6)-40	P0[10] UART2 - TX
	J2(J6)-41   P0[11] UART2 - RX	Debug Interface
  
  	~~J2(J6)-25   P0[27] I2C - SDA~~
  	~~J2(J6)-26   P0[28] I2C - SCI 	I2C connection to SRS Submodule~~
	
	PAD8(PAD17)	   	P0[19] I2C - SDA
	PAD2(PAD18)   	P0[20] I2C - SCI 	I2C connection to SRS Submodule
	
  
  	Optional:
  	J2(J6)-21   P0[2] UART0 - TX
  	J2(J6)-22   P0[3] UART0 - RX  	receiving GPS NMEA records ("$GPRMC,...")
  
  
