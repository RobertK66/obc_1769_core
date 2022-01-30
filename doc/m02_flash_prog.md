How to program the OBCs Flash
=============================

A. With MCUXpresso IDE
----------------------

- Connect a LPC-Link2 probe to the JTAG pins of the OBC Y- connector

- install and run a MCUXpresso developer IDE

- make an enpty C project - choose LPC1769 as target CPU. Do not choose any boards

- start the GUI Flash Tool from the MCUXpresso toolbar 

![Step1](pic/Flash1.jpg)

- select the axf file you want to program into your OBC

![Step2](pic/Flash2.jpg)

- run the tool and check if program was successful.

![Step3](pic/Flash3.jpg)


B. With provided Windows BAT file
---------------------------------

- Download a release and copy the provided FlashOBC.bat file (together with the ClimbOBCR_x_x_x.axf) file into any local directory.
- Check if your local MCUXpresso instalation is used by reviewing/adapting the first line of this BAT file
- Open A CMD window with evelated user rights!
- Call FlashOBC.BAT <ClimbObcRxxx> with the correct axf file name as parameter
- If everythink goes ok, your terminal should somewhere contain the lines: "Loading 'ClimbObcR_0_8_2.axf' ELF 0x00000000 len 0x878C" and "Loaded 0x878C bytes in 181ms (about 191kB/s)"




