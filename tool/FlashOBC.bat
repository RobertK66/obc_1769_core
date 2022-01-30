set MCU_TOOL_BIN=C:\nxp\MCUXpressoIDE_11.4.0_6237\ide\plugins\com.nxp.mcuxpresso.tools.bin.win32_11.4.0.202108051708\binaries
@ECHO OFF


if exist %MCU_TOOL_BIN% (
  echo MCU installation found!
) else (
  echo MCU installation NOT FOUND!
  echo Pls define your local installed MCUXpresso's binary tools directory in the first line of this bat file. This heavily depends on your local used version of MCUXperesso. 
  EXIT /B 1
)

CACLS "%SYSTEMROOT%\system32\config\system" >nul
IF ERRORLEVEL 1 (
	ECHO *** This batch file requires elevated privileges! Start with Admin rights! ***
	EXIT /B 1
)
call %MCU_TOOL_BIN%\boot_link2 >nul
@ECHO On

TIMEOUT -T 3 /NOBREAK

%MCU_TOOL_BIN%\crt_emu_cm_redlink -flash-load-exec %1 -vendor=NXP -pLPC1769

pause

