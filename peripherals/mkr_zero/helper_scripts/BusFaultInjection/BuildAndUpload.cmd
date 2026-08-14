@echo off
setlocal

rem BusFaultInjection - wedges the I2C bus on purpose and proves recovery works.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem Needs BOTH libraries: vendor\Wire supplies the patch under test, lib\
rem supplies I2CBus/GPSFunctions/DataDictionary.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
for %%I in ("%~dp0..\..\lib") do set "LIB_DIR=%%~fI"
for %%I in ("%~dp0..\..\vendor\Wire") do set "WIRE_DIR=%%~fI"
rem Pinned Bosch BNO055 driver - see ..\..\vendor\BNO055\PATCHES.md.
for %%I in ("%~dp0..\..\vendor\BNO055") do set "BNO_DIR=%%~fI"
rem Not used by this sketch. Needed because --library lib\ compiles EVERY source
rem in lib\, including SDFunctions.cpp and the CAN files, and each fails its own
rem vendoring marker without these.
for %%I in ("%~dp0..\..\vendor\CANBus") do set "CAN_DIR=%%~fI"
for %%I in ("%~dp0..\..\vendor\SdFat") do set "SDFAT_DIR=%%~fI"

set "FQBN=arduino:samd:mkrzero"
set "CORE_REQUIRED=1.8.14"
set "WARN=--warnings all"

set "CORE_FOUND="
for /f "tokens=2" %%V in ('arduino-cli core list ^| findstr /b /c:"arduino:samd"') do set "CORE_FOUND=%%V"
if not "%CORE_FOUND%"=="%CORE_REQUIRED%" (
    echo ERROR: arduino:samd %CORE_REQUIRED% is required, found "%CORE_FOUND%".
    exit /b 1
)

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%BNO_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%BNO_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
