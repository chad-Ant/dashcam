@echo off
setlocal

rem BNO055BringUp - Phase 2 gate for the staged BNO055 bring-up.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem Runs the bring-up state machine from loop() and reports the two numbers that
rem decide whether it is genuinely non-blocking: the longest single tick, and the
rem longest gap between loop passes. The 650 ms reset wait happens during the
rem run, so a blocking implementation would show ~650000 us in both.
rem
rem The --library flags are REQUIRED, not a convenience:
rem
rem   vendor\BNO055  Pinned Bosch driver. WITHOUT this flag arduino-cli resolves
rem                  <BNO055.h> from the global Arduino libraries folder, whose
rem                  bno055_init() overwrites the device address with 0x28 - the
rem                  ALTERNATIVE address, not the default - so on a GY breakout
rem                  every read goes nowhere. lib\BNO055Transport.h fails the
rem                  build on a missing marker rather than link the unpinned
rem                  copy. See vendor\BNO055\PATCHES.md.
rem
rem   vendor\Wire    Patched SAMD Wire. Stock TwoWire::requestFrom() reads an
rem                  uninitialised busOwner on 1-byte transfers, and single-byte
rem                  register reads are nearly every transaction this sketch
rem                  makes. See vendor\Wire\README.md.
rem
rem   lib\           I2CBus.cpp, BNO055Transport.cpp and BNO055Init.cpp are
rem                  compiled from here. Arduino compiles a sketch folder but
rem                  never an arbitrary lib\.
rem
rem   vendor\SdFat   Not used by this sketch. Needed because --library lib\
rem   vendor\CANBus  compiles EVERY source in lib\, including SDFunctions.cpp
rem                  and the CAN files, and each fails its own vendoring marker
rem                  without these. Same reason CanMapSelfTest carries them.
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
for %%I in ("%SKETCH_DIR%\..\..\vendor\BNO055") do set "BNO_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\CANBus") do set "CAN_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\SdFat") do set "SDFAT_DIR=%%~fI"

set "FQBN=arduino:samd:mkrzero"
set "CORE_REQUIRED=1.8.14"
set "WARN=--warnings all"

set "CORE_FOUND="
for /f "tokens=2" %%V in ('arduino-cli core list ^| findstr /b /c:"arduino:samd"') do set "CORE_FOUND=%%V"
if not "%CORE_FOUND%"=="%CORE_REQUIRED%" (
    echo.
    echo ERROR: arduino:samd %CORE_REQUIRED% is required, found "%CORE_FOUND%".
    exit /b 1
)

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%BNO_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%BNO_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
