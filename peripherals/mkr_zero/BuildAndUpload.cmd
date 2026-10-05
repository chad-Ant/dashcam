@echo off
setlocal

rem MKR Zero telemetry master - build / flash with arduino-cli.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem   BuildAndUpload.cmd COM5 amg   ...with the IMU in its raw (AMG) mode
rem   BuildAndUpload.cmd COM5 selftest
rem                                 BENCH build: the raw CAN stream's self-test
rem
rem "selftest" defines DASHCAM_CAN_STREAM_SELFTEST=2400: while the CAN drain is
rem armed its timer interrupt also synthesizes 2400 frames a second (id 0x7F0,
rem a sequence number in the data), with no bus, so the USB stream's throughput
rem can be measured on the bench. See build_and_upload.sh for the details and
rem for "selftest=N" (another rate) and "amg selftest" together; this script
rem takes one option, "amg" OR "selftest" - cmd.exe splits "selftest=N" at the
rem "=" and quoting a property with two defines is not worth the risk here.
rem NEVER flash a selftest build to the car.
rem
rem The optional "amg" argument defines DASHCAM_IMU_MODE_AMG, which selects
rem IMUSampleMode::Raw at bring-up instead of the production IMUPLUS. The mode is
rem chosen once in setup() and no command changes it at runtime, so this is the
rem only way to exercise the raw path end to end. WITHOUT the argument the build
rem is bit-for-bit the production one.
rem
rem It sets compiler.cpp.extra_flags rather than build.extra_flags: the latter is
rem where the SAMD platform keeps {build.usb_flags}, and overriding it would drop
rem the USB VID/PID defines and produce a board that does not enumerate.
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
rem The MKR's USB port is a PRODUCTION data path, not just a development
rem link: next to the console it streams every CAN frame to the Orin as
rem "F ..." lines, with an "FS ..." stats line each second (see
rem lib\CANRawStream.h). Stop whatever reads that port in the car before an
rem upload, and expect the raw stream to be missing while it runs.
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

rem "amg" and "selftest" are accepted in EITHER position, so a compile-only
rem build does not need an empty first argument - a shell that collapses ""
rem would otherwise pass the option as the PORT and try to upload to a device of
rem that name. One option per build; "selftest" wins if both are given.
rem Each set is its own line: grouping them inside parentheses needs delayed
rem expansion to read back, which is a well-known way to get an empty variable.
set "MODE="
if /i "%~1"=="amg" set "MODE=amg"
if /i "%~2"=="amg" set "MODE=amg"
if /i "%~1"=="selftest" set "MODE=selftest"
if /i "%~2"=="selftest" set "MODE=selftest"
set "PORT=%~1"
if /i "%~1"=="amg" set "PORT="
if /i "%~1"=="selftest" set "PORT="
set "EXTRA="
if "%MODE%"=="amg" set "EXTRA=--build-property compiler.cpp.extra_flags=-DDASHCAM_IMU_MODE_AMG"
if "%MODE%"=="amg" echo Building with the IMU in AMG ^(raw^) mode.
if "%MODE%"=="selftest" set "EXTRA=--build-property compiler.cpp.extra_flags=-DDASHCAM_CAN_STREAM_SELFTEST=2400"
if "%MODE%"=="selftest" echo Building the BENCH self-test: 2400 synthetic CAN frames/s. Never flash this to the car.

if "%PORT%"=="" (
    arduino-cli compile %WARN% %EXTRA% --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% %EXTRA% --upload --port "%PORT%" --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
