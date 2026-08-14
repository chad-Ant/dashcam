@echo off
setlocal

rem MKR Zero telemetry master - build / flash with arduino-cli.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem The --library flags are REQUIRED, not a convenience.
rem
rem   lib\        Arduino compiles a sketch's own folder and its src/ subfolder,
rem               but never an arbitrary lib/. Without this the sketch still
rem               compiles (the headers resolve) and then fails at link with
rem               "undefined reference" to every function in lib/.
rem
rem   vendor\Wire A patched copy of the SAMD core's Wire. The stock
rem               TwoWire::requestFrom() reads an uninitialised busOwner on every
rem               1-byte transfer, which can drop the STOP or report 0 bytes for
rem               a transfer that worked - and 1-byte register reads are the most
rem               common transaction on the shared IMU/GNSS bus. An explicitly
rem               supplied library outranks the platform-bundled copy. If this
rem               flag is ever lost, lib\I2CBus.h fails the build on a missing
rem               marker rather than silently linking the buggy one.
rem               See vendor\Wire\README.md.
rem
rem   vendor\CANBus  A patched copy of timurrrr's arduino-CAN fork. WITHOUT this
rem               flag arduino-cli resolves <CAN.h> from the global Arduino
rem               libraries folder - an unpinned directory nothing version-checks
rem               - and every fix in vendor\CANBus silently stops reaching the
rem               firmware. The upstream copy hangs forever in endPacket() on a
rem               stuck bus, overflows its 8-byte RX buffer on a DLC above 8,
rem               truncates 11-bit filters to 8 bits, and puts the controller in
rem               Configuration mode when asked for Listen-Only. lib\OBD2Functions.h
rem               fails the build on a missing marker if this flag is ever lost.
rem               See vendor\CANBus\PATCHES.md.
rem
rem This also depends on mkr_zero.ino including the headers by BARE name
rem ("OBD2Functions.h", not "lib/OBD2Functions.h"): a path-qualified include is
rem treated as a plain relative file and never binds to the library, which
rem reproduces the same link failure even with --library present.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\lib") do set "LIB_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\vendor\Wire") do set "WIRE_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\vendor\CANBus") do set "CAN_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\vendor\SdFat") do set "SDFAT_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\vendor\BNO055") do set "BNO_DIR=%%~fI"

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
    echo        vendor\Wire is a patched copy of that core's Wire library and
    echo        depends on its SERCOM API. Install with:
    echo            arduino-cli core install arduino:samd@%CORE_REQUIRED%
    echo        If you are deliberately moving to a newer core, re-vendor and
    echo        re-verify the patch first - see vendor\Wire\README.md.
    exit /b 1
)

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%BNO_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%BNO_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
