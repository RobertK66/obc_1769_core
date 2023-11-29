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

 
Note: not all commands are available on all tagret hardware.

| command | params                    | description                                                                                                      | obc | dev |
|---------|---------------------------|------------------------------------------------------------------------------------------------------------------|-----|-----|
| ‘s’     | \-                        | reads all on board sensor values. triggers the “\<sens\>-0x01” event.                                            | x | |
| ‘c’     | \-                        | power off the SD-Card                                                                                            | x | |
| ‘C’     | \-                        | power on the SD-Card                                                                                             | x | |
| ‘R’     | \<blockNr\>               | reads one block from SD-Card and triggers the “\<sys\>-data” event.                                              | x | |
| ‘r’     | \<c\>\<adr\>\<len\>       | reads a number \<len\> bytes from \<adr\> of mram chip number \<c\> (0...5). It triggers a “\<sys\>-data” event. | x | |
| ‘w’     | \<c\>\<adr\>\<b\>\<len\>  | writes the data byte \<b\> for \<len\> times starting at \<adr\> of mram chip number \<c\> (0...5).              | x | |
| 'p'     | \<abcd\|ABCD\>            | powers on or off one or more sidepanels. e.g "p ABcD" powers 3 of the 4 sidepanels. Blue LED shows on and goes off if current limiter detects short circuit |  x | |
| ‘O’     | \<name\>                  | sets the hardware instance name                                                                                  | x | |
| 'N'     | \<name\>                  | sets the sd-card name                                                                                            | x | |
| 'i'     | \-                        | gives general information about obc board, versions, ....                                                        | x | |
| 'T'     | \-                        | gives the current UTC time in RTC and TLE (juliandayfraction) format                                             | x | x |
| 't'     | \<date\> \<time\>         | sets the UTC time to d´the given date / time. Format: yyyyMMdd hhmmss                                            | x | x |
| ---     | ---                       | ---                                                                                                              | x | |
| 'h'     | \<pinIdx\> \<mode\>       | sets an GPIO pin to mode (0: initVal, 1: high, 2: low, 3: slow blink, 4: fast osz.)                              | x | |
| 'm'     | \<pinIdxIn\> [\<pinIdxOut\>] | mirrors the GPI input pin to the GPIO output pin 															 | x | |
| '5'     |  | Thruster all register read request.															                                             | x | |
| '6'     | \<register index\>  | Read request to single thruster register.														                         | x | |
| '7'     | \<register index\> [\<value\>] | Set request for a single thruster register. 															 | x | |
| 'S'     | \<srscmd\> [\<value\>] | sends a command to the SRS submodule. 					 |  | x |
|         | p  | SRS command 'Power Off' (only commands over I2C if submodule was ON) 		 |  | x |
|         | P  | SRS command 'Power On' (only commands over I2C if submodule was OFF) 		 |  | x |
|         | t  | Syncs the SRS to current OBC timestamp. (set eith 't' or over GPS-UART)     |  | x |



Valid pinIdx for command 'h' and 'm' can be found in the hardware abstraction include file in source code..... 

Events
------

Events are sent by using a frame start/end Character of **0x7e.** If a databyte
of 0x7e (or 0x7d) has to be transmitted this byte is escaped by using the escape
char: **0x7d** and the escaped data byte is XOR-ed with **0x20**.


So one frame looks like this:

**0x7e \<Mod\> \<SevId\> \<byte0\> \<byte1\> ... 0x7e**

where

-   \<Mod\> is the module number - the originator of the event.

-   \<SevId\> is composed as 2+6 bit: sseeeeee.

    -   ss: the 2 high bits hold the severity (00 - Info, 01 - Warning, 10 -
        Error, 11 - Fatal)

    -   eeeeee: the 6 low bits are the event ID. This ID is specific to each
        module.

 

### Modules

| module number | module name             |
|---------------|-------------------------|
| 0x00          | climb application layer |
| 0x01          | timer                   |
| 0x02          | sensors                 |
| 0x03          | memory                  |
| 0x80          | mram                    |
| 0x81          | sdcard                  |

 

### Events

| module-nr/event-id | format  | description                |
|--------------------|---------|----------------------------|
| 0x00 / 0x02        | n bytes | raw data                   |
| 0x00 / 0x03        | n chars | a simple ASCII string      |
|--------------------|---------|----------------------------|
| 0x02 / 0x01        | 6 float | sensor values:             |
|                    |   [0]   | supply voltage in V        |
|                    |   [1]   | current OBC   in mA        |
|                    |   [2]   | current side panels in mA  |
|                    |   [3]   | temperature (LM19) in °C   |
|                    |   [4]   | temperature (SHT3X) in °C  |
|                    |   [5]   | humidity in %   			|
|--------------------|---------|----------------------------|
| 0x01 / 0x03        |         | UTC time was synchronized  |
|                    | uint32  | current reset number       |
|                    | double  | old UTC Offset             |
|                    | double  | new UTC Offset             |
|                    | byte    | Sync Source (1 GPS, 2 Cmd, 3 RTC) |
|--------------------|---------|----------------------------|
