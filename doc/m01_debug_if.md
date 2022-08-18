OBC Debug Interface
===================

General description
-------------------

The OBC debug interface is a serial communication link which is available at the
umbilical connector of the cubesat. With this interface all operations prior to
launch (and integration into the launch box) can be executed via connected
ground equipment.

 

UART-specifications

| baud rate | databits | parity | stop |
|-----------|----------|--------|------|
| 9600      | 8        | none   | 1    |
 

Commanding
----------
On Rx Line the OBC reacts to received commands. The format of a command is:

**\<cc\> [par1 [par2 [...]]]\<LF\>**

where

-   \<cc\> is a single command character. e.g. ‘s’ for the “Read
    Sensors”-command.

-   par1...parX are optional parameters separated by a single blank ‘ ‘ (0x20).
    Count and type of parameter is determined by specific commands. Either a
    numeric value or a string can be used. (At this moment there is no string
    marker (”) usable, so a string with blanks can not be used as a single par!
    -\> use ‘_’ or ‘-’ instead.)

-   \<LF\> is the line feed character: **0x0a**.

 

| command | params                    | description                                                                                                      |
|---------|---------------------------|------------------------------------------------------------------------------------------------------------------|
| ‘s’     | \-                        | reads all on board sensor values. triggers the “\<sens\>-0x01” event.                                            |
| ‘c’     | \-                        | power off the SD-Card                                                                                            |
| ‘C’     | \-                        | power on the SD-Card                                                                                             |
| ‘R’     | \<blockNr\>               | reads one block from SD-Card and triggers the “\<sys\>-data” event.                                              |
| ‘r’     | \<c\>\<adr\>\<len\>       | reads a number \<len\> bytes from \<adr\> of mram chip number \<c\> (0...5). It triggers a “\<sys\>-data” event. |
| ‘w’     | \<c\>\<adr\>\<b\>\<len\>  | writes the data byte \<b\> for \<len\> times starting at \<adr\> of mram chip number \<c\> (0...5).              |
| 'p'     | \<abcd\|ABCD\>            | powers on or off one or more sidepanels. e.g "p ABcD" powers 3 of the 4 sidepanels. Blue LED shows on and goes off if current limiter detects short circuit | 
| ‘O’     | \<name\>                  | sets the hardware instance name                                                                                  |
| 'N'     | \<name\>                  | sets the sd-card name                                                                                            |
| 'i'     | \-                        | gives general information about obc board, versions, ....                                                        |
| 'T'     | \-                        | gives the current UTC time in RTC and TLE (juliandayfraction) format                                             |
| 't'     | \<date\> \<time\>         | sets the UTC time to d´the given date / time. Format: yyyyMMdd hhmmss                                            |
| ---     | ---                       | ---                                                                                                              |
| 'h'     | \<pinIdx\> \<mode\>       | sets an GPIO pin to mode (0: initVal, 1: high, 2: low, 3: slow blink, 4: fast osz.)                              |
| 'm'     | \<pinIdxIn\> [\<pinIdxOut\>] | mirrors the GPI input pin to the GPIO output pin 															 |
| '5'     |  | Thruster all register read request.															                                             |
| '6'     | \<register index\>  | Read request to single thruster register.														                         |
| '7'     | \<register index\> [\<value\>] | Set request for a single thruster register. 															 |

Valid pinIdx for command 'h' and 'm' can be found in the hardware abstraction include file in source code..... 

All commands do work during the atostarted Radiation Tests. But note that not alll events triggered by commands are shouwn as textual outputs. 
For checking the memory tests the 'r' and 'w' commands are working. 


Events
------

~~Events are sent by using a frame start/end Character of **0x7e.** If a databyte
of 0x7e (or 0x7d) has to be transmitted this byte is escaped by using the escape
char: **0x7d.** ~~

Some events are translated to textual Output on the UART. The tests are autostarted after reset and the outputs are done with a fixed timimng schedule.

