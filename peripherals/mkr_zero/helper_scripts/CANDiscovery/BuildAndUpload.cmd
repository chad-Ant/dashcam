@echo off
setlocal

rem CANDiscovery - passive CAN census with a GNSS speed cross-check.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem This sketch USED to be self-contained with no --library flags. It no longer
rem is, because it now cross-checks the decoded CAN wheel speed against GNSS and
rem reuses the project's own GNSS stack to do it. A second, simpler GNSS path
rem written for a helper sketch would be a second thing that can be wrong about
rem the receiver - and the receiver is the reference the calibration rests on.
rem It still MODIFIES no production source; it only reads lib\.
rem
rem   lib\        Arduino compiles a sketch's own folder and its src\ subfolder,
rem               but never an arbitrary lib\. Without this the sketch still
rem               compiles (the headers resolve) and then fails at link with
rem               "undefined reference" to every function in lib\.
rem
rem   vendor\Wire A patched copy of the SAMD core's Wire. The stock
rem               TwoWire::requestFrom() reads an uninitialised busOwner on every
rem               1-byte transfer, which can drop the STOP or report 0 bytes for
rem               a transfer that worked - and 1-byte register reads are the most
rem               common transaction on the shared GNSS/IMU bus. If this flag is
rem               ever lost, lib\I2CBus.h fails the build on a missing marker
rem               rather than silently linking the buggy one.
rem               See vendor\Wire\README.md.
rem
rem Includes must be by BARE name ("GPSFunctions.h", not "lib/GPSFunctions.h"):
rem a path-qualified include is treated as a plain relative file and never binds
rem to the library, reproducing the same link failure even with --library set.
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
    exit /b 1
)

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
