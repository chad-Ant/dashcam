@echo off
setlocal

rem IMUValidation - bring-up and GPS coexistence test for the LSM6DSOX + LIS3MDL.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem   BuildAndUpload.cmd /nowire    negative test: build WITHOUT vendor\Wire and
rem                                 expect the I2CBus.h marker guard to fail
rem
rem This helper DOES pass --library, unlike CANDiscovery: its whole purpose is to
rem exercise peripherals/mkr_zero/lib (IMUFunctions and GPSFunctions on one bus),
rem so building against a copy would validate the wrong code. As in the main
rem sketch, the headers must be included by BARE name ("IMUFunctions.h", not
rem "lib/IMUFunctions.h") or the .cpp files never compile and the link fails with
rem "undefined reference".
rem
rem vendor\Wire carries the 1-byte requestFrom fix; see vendor\Wire\README.md.
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
rem Pinned Bosch BNO055 driver. Without it arduino-cli resolves <BNO055.h> from
rem the global Arduino libraries folder, whose bno055_init() overwrites the
rem device address with 0x28 - the ALTERNATIVE address, not the default - so on a
rem GY breakout every read goes nowhere. See ..\..\vendor\BNO055\PATCHES.md.
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
    echo        See ..\..\vendor\Wire\README.md.
    exit /b 1
)

rem Negative test. Proves the guard actually bites: without the vendored Wire the
rem build MUST fail on the missing DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX marker. A
rem guard nobody has watched fail is a guard nobody knows works.
if /i "%~1"=="/nowire" (
    echo Negative test: building WITHOUT vendor\Wire, expecting the marker guard to fail...
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%BNO_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%" >nul 2>&1
    if errorlevel 1 (
        echo PASS - build correctly refused without the patched Wire.
        exit /b 0
    )
    echo FAIL - build SUCCEEDED without vendor\Wire. The marker guard in
    echo        lib\I2CBus.h is not protecting anything.
    exit /b 1
)

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%BNO_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%BNO_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
