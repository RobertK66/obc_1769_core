
OBC Debug Interface
===================

General description
------------------

The Obc Debug interface is a Serial communication link which is available at the
Umbilical connector of the cubesat. With this interface all operations prior to
launch (and integration into the launch box) can be executed via connected
Ground equipment.

UART-specifications

| baud rate | 9600  |
|-----------|-------|
| databits  | 8     |
| parity    | none  |
| stop      | 1     |

 

Commanding
----------

On Rx Line the OBC reacts to received commands. The format of a command is:

 

**\<cc\> [par1 [par2 [...]]]\<LF\>**

 

where \<cc\> is a single command character. e.g. ‘s’ for the “Read
Sensors”-command.

par1...parX are optional parameters separated by a single blank ‘ ‘ (0x20).
Count and type of parameter is determined by specific commands. Either a numeric
value or a string can be used. (At this moment there is no string marker (”)
usable, so a string with  blanks can not be used as a single par! -\> use ‘_’ or
‘-’ instead.)

\<LF\> is the line feed character: **0x0a**.

 

| command | params              | description                                                                                                      |
|---------|---------------------|------------------------------------------------------------------------------------------------------------------|
| ‘s’     | \-                  | reads all on board sensor values. triggers the “\<sens\>-0x01” event.                                            |
| ‘R’     | \<blockNr\>         | reads one block from SD-Card and triggers the “\<sys\>-data” event.                                              |
| ‘r’     | \<c\>\<adr\>\<len\> | reads a number \<len\> bytes from \<adr\> of mram chip number \<c\> (0...5). It triggers a “\<sys\>-data” event. |

 

 

Events
------

 

Events are sent by using a frame start/end Character of **0x7e.**

If a databyte of 0x7e (or 0x7d)  has to be transmitted this byte is escaped by
using the escape char: **0x7d.**

 

So one frame looks like this:

 

0x7e \<Mod\> \<SevId\> \<byte0\> \<byte1\> ... \<0x7e\>

 

where \<Mod\> is the module number - the originator of the event.

\<SevId\> is composed as sseeeeee.

ss: the 2 high bits hold the sevirity (00 - Info, 01 - Warning, 10 - Error, 11 -
Fatal)

eeeeee: the 6 low bits are the event ID. This ID is specific to each module.

 

| \<Mod\> | eventid | format   | description                                                          |
|---------|---------|----------|----------------------------------------------------------------------|
| timer   | 1       | xx xx xx | The event Eins from timer tbd .... not yet ready to be described.... |
|         |         |          |                                                                      |
