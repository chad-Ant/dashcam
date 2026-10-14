@echo off
setlocal

rem ES3C28P dashboard - compile / upload with arduino-cli.
rem   BuildAndUpload.cmd          compile only
rem   BuildAndUpload.cmd COM7     compile and upload to COM7

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\lib\es3Graphics") do set "GRAPHICS_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\lib\es3Storage") do set "STORAGE_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\lib\es3Audio") do set "AUDIO_DIR=%%~fI"

arduino-cli lib install --no-deps "JPEGDEC@1.8.4"
if errorlevel 1 exit /b 1
if not defined ES3_USB_MODE set "ES3_USB_MODE=hwcdc"

rem ES3C28P N16R8, native USB Serial/JTAG CDC.
set "FQBN=esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi,USBMode=%ES3_USB_MODE%,CDCOnBoot=cdc"
set "WARN=--warnings all"

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%STORAGE_DIR%" --library "%GRAPHICS_DIR%" --library "%AUDIO_DIR%" --library "%SKETCH_DIR%\lib\es3Board" --library "%SKETCH_DIR%\lib\es3UsbLink" --library "%SKETCH_DIR%\lib\es3Video" --library "%SKETCH_DIR%\lib\es3Map" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%STORAGE_DIR%" --library "%GRAPHICS_DIR%" --library "%AUDIO_DIR%" --library "%SKETCH_DIR%\lib\es3Board" --library "%SKETCH_DIR%\lib\es3UsbLink" --library "%SKETCH_DIR%\lib\es3Video" --library "%SKETCH_DIR%\lib\es3Map" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
