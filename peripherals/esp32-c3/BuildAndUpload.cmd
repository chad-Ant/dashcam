@echo off
setlocal

rem ESP_Sentinel telemetry bridge - build / flash with arduino-cli.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM7       compile and upload to COM7
rem
rem Sources: esp32-c3.ino (empty marker) + src/main.cpp, with lib/commLink and
rem lib/hostLink supplied as libraries. arduino-cli compiles src/ recursively
rem but does NOT reach into lib/, so both must be passed with --library.
rem
rem NOTE: keep this file plain ASCII. cmd.exe parses it in the OEM codepage, so
rem a stray UTF-8 character (an em dash in a comment is enough) shifts the byte
rem stream and the interpreter starts executing fragments of its own comments.

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\lib\commLink") do set "COMMLINK_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\lib\hostLink") do set "HOSTLINK_DIR=%%~fI"

rem CDCOnBoot=cdc is REQUIRED, not a preference: Serial must be the native USB
rem port wired to the Jetson. Without it Serial becomes UART0 on GPIO20/21, the
rem pins commLink already drives for the MKR Zero link, and hostLink.h fails the
rem build with an #error rather than letting that ship.
rem DebugLevel=none keeps the core's log macros off the binary USB channel.
set "FQBN=esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app,CPUFreq=160,FlashMode=qio,FlashFreq=80,FlashSize=4M,DebugLevel=none,UploadSpeed=921600"

rem This firmware is held to -Wall -Wextra; surface every warning.
set "WARN=--warnings all"

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%COMMLINK_DIR%" --library "%HOSTLINK_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%COMMLINK_DIR%" --library "%HOSTLINK_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
