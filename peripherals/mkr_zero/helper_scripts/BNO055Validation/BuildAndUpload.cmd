@echo off
setlocal

rem BNO055Validation - Phase 1 gate for the BNO055 transport.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem Proves the transport before anything is built on it: which address the part
rem is really at, whether CHIP_ID reads 0xA0 repeatedly with zero faults, and
rem whether the shared bus survives a part the datasheet says stretches the
rem clock. No fusion, no modes - those come later, and this sketch is what lets
rem you rule the transport out when they misbehave.
rem
rem The --library flags are REQUIRED, not a convenience:
rem
rem
rem   vendor\Wire    Patched SAMD Wire. Stock TwoWire::requestFrom() reads an
rem                  uninitialised busOwner on 1-byte transfers, and single-byte
rem                  register reads are the most common transaction this driver
rem                  makes. See vendor\Wire\README.md.
rem
rem   lib\           I2CBus.cpp (bus recovery, stuck-line reporting, watchdog)
rem                  and BNO055Transport.cpp are compiled from here. Arduino
rem                  compiles a sketch folder but never an arbitrary lib\.
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
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
