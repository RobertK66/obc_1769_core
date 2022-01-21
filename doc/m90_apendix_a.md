Apendix A / Archtectural Decisions and Reasoning
================================================

Hardware and CPU
----------------
Decission to take the own OBC design with LPC1769 is based on
- Experience from Pegasus (the Climb predecessor project)
- Excelent radiation robustnes of this design (proofed in orbit and other ground tests).

Reconsider if
- more complex caclulations and processes needed then the LPC1769 can deliver. 


Programming Language
--------------------
Decision to take pure ANSI C is based on 
- Mixed mode of using C and C++ was a main hurdle for a clean build in the Pegasus project.
- C++ is considered a to high entry level for students new to embedded software development.
- Rust was considered (at an late stage after development already started) but discarded because lack of compiler and tools for our CPU.

Reconsider if
- more complex calculations other than low level hardware control and 'simple' mission control is needed. (e.g. making suporting own ADCS algorithm)

Library & IDE Usage
-------------------
Decision to take MCUXpresso based on
- The only direct supported IDE from NXP (for this CPU).

Decision to take Redlib(none) based on
- more used and mature then the other possibilities ('newlib')
- (none) lowest memory footprint and no dependencies to undocumented code. ("it excludes low-level functions for all file-based I/O and some string and memory handling functions.")

Reconsider if
- better IDE support and cost free usage for other vendors libraries (e.g. SEGGER)

Decision to reuse and rewrite the NXP provided "LPCOpen" Library as its own "ado-chip-175x-6x"
- LPCOpen is an 'old' and depricated version of CPU and hardware abstraction from NXP. No new developments since the release 03/13/2014 (v2.10).
- MCUXpresso projects use the LPCOpen layout and always include this 'Library' as source code in its own project. So its easy to replace and change this to own code.
- There where already errors found and fixed during the pegasus project.

Reconsider if
- LPC1769 gets included and supported in some better and new 'Embedded Software Development Communities". (e.g. MCUXpressos new SDKs)

Usage of OS/RTOS
----------------
Decision of not using available RTOS implementation for LPC1769 based on
- overhead of task switching vs. decoupling of 'modules' considered to high for this application.
- complexity added for development and debugging ( another hurdle for students new to embedded software dev)

Reconsider if
- own solution gets to comlex and/or also introduces comparable overhead.

