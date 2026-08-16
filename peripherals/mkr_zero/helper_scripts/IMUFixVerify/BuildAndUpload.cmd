@echo off
setlocal

rem IMUFixVerify - regression tests for the IMU review fixes.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem Then open a serial monitor at 115200. Every test reports PASS, FAIL or SKIP
rem and the sketch prints a summary; groups whose hardware is absent SKIP rather
rem than FAIL, so a bare board still exercises the pure-logic tests.
rem
rem WARNING: with an SD card fitted this rewrites bno055.cal. The original is
rem copied into RAM and written back at the end, and the sketch says so loudly if
rem the restore fails.
rem
rem The --library flags are REQUIRED, not a convenience:
rem
rem   vendor\Wire    Patched SAMD Wire. Stock TwoWire::requestFrom() reads an
rem                  uninitialised busOwner on 1-byte transfers, which is nearly
rem                  every transaction the page-recovery test makes.
rem                  See vendor\Wire\README.md.
rem
rem   lib\           IMUFunctions.cpp, BNO055*.cpp, SDFunctions.cpp and
rem                  CommunicationFunctions.cpp are compiled from here. Arduino
rem                  compiles a sketch folder but never an arbitrary lib\.
rem
rem   vendor\SdFat   SDFunctions.cpp needs SdFat. vendor\CANBus is not used by
rem   vendor\CANBus  this sketch but --library lib\ compiles EVERY source in
rem                  lib\, including the CAN files, and each fails its own
rem                  vendoring marker without the flag. Same reason
rem                  BNO055BringUp carries them.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\lib") do set "LIB_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\Wire") do set "WIRE_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\CANBus") do set "CAN_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\SdFat") do set "SDFAT_DIR=%%~fI"

set "FQBN=arduino:samd:mkrzero"
set "CORE_REQUIRED=1.8.14"
set "WARN=--warnings all"

rem The vendored Wire calls the core's private SERCOM API, which carries no
rem stability guarantee across core releases. Refuse to build against a core it
rem was never tested with rather than emit a binary that merely happened to link.
set "CORE_FOUND="
for /f "tokens=2" %%V in ('arduino-cli core list ^| findstr /b /c:"arduino:samd"') do set "CORE_FOUND=%%V"
if not "%CORE_FOUND%"=="%CORE_REQUIRED%" (
    echo.
    echo ERROR: arduino:samd %CORE_REQUIRED% is required, found "%CORE_FOUND%".
    echo        Install with:
    echo            arduino-cli core install arduino:samd@%CORE_REQUIRED%
    exit /b 1
)

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
