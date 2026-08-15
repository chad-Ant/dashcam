@echo off
setlocal

rem WireRegression - proves the vendored SAMD Wire requestFrom() patch works.
rem
rem   BuildAndUpload.cmd            compile only        (vendored Wire)
rem   BuildAndUpload.cmd COM5       compile and upload  (vendored Wire)
rem   BuildAndUpload.cmd /stock            compile only        (STOCK core Wire)
rem   BuildAndUpload.cmd /stock COM5       compile and upload  (STOCK core Wire)
rem
rem /stock is the CONTROL arm of the experiment. It builds this same sketch
rem against the unpatched core Wire so the two can be compared without anyone
rem hand-editing a guard - a "temporary" edit being exactly the kind of thing
rem that gets committed by accident. It works by defining
rem DASHCAM_WIRE_STOCK_CONTROL, the single sanctioned bypass of the marker guard
rem in lib\I2CBus.h, and the sketch then prints "STOCK CONTROL / marker absent"
rem on every run so the two binaries can never be confused. No production build
rem script defines it; the production guard stays fail-closed.
rem
rem Both modes define DASHCAM_WIRE_INSTRUMENT, which enables the test-only
rem one-byte transfer counters. Production scripts never define it.
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
rem Not used by this sketch's tests, but --library lib\ compiles EVERY source in
rem lib\ - including the BNO055 transport, SDFunctions and the CAN files - and
rem each fails its own vendoring marker without these.
for %%I in ("%~dp0..\..\vendor\CANBus") do set "CAN_DIR=%%~fI"
for %%I in ("%~dp0..\..\vendor\SdFat") do set "SDFAT_DIR=%%~fI"
set "VENDOR_LIBS=--library "%CAN_DIR%" --library "%SDFAT_DIR%""

set "FQBN=arduino:samd:mkrzero"
set "CORE_REQUIRED=1.8.14"
set "WARN=--warnings all"
set "INSTRUMENT=-DDASHCAM_WIRE_INSTRUMENT=1"

set "CORE_FOUND="
for /f "tokens=2" %%V in ('arduino-cli core list ^| findstr /b /c:"arduino:samd"') do set "CORE_FOUND=%%V"
if not "%CORE_FOUND%"=="%CORE_REQUIRED%" (
    echo ERROR: arduino:samd %CORE_REQUIRED% is required, found "%CORE_FOUND%".
    exit /b 1
)

if /i "%~1"=="/stock" (
    echo.
    echo *** STOCK CONTROL BUILD - unpatched core Wire, guard bypassed on purpose ***
    echo.
    set "PORT=%~2"
    rem Clean build so nothing from a vendored build can be reused, and verbose
    rem so the resolved Wire path is captured in the log below.
    if "%~2"=="" (
        arduino-cli compile %WARN% --clean --verbose --fqbn "%FQBN%" --build-property "compiler.cpp.extra_flags=-DDASHCAM_WIRE_STOCK_CONTROL=1" %VENDOR_LIBS% --library "%LIB_DIR%" "%SKETCH_DIR%" > "%TEMP%\wireregression_stock.log" 2>&1
    ) else (
        arduino-cli compile %WARN% --clean --verbose --upload --port "%~2" --fqbn "%FQBN%" --build-property "compiler.cpp.extra_flags=-DDASHCAM_WIRE_STOCK_CONTROL=1" %VENDOR_LIBS% --library "%LIB_DIR%" "%SKETCH_DIR%" > "%TEMP%\wireregression_stock.log" 2>&1
    )
    set "RC=%ERRORLEVEL%"
    rem NOTE: no parentheses in echo text inside an if-block - an unescaped ")"
    rem closes the block early and cmd then chokes on the next token.
    rem Surface failures rather than burying them: the whole build output goes to
    rem the log, so without this a broken control build looks like a quiet success
    rem with a couple of informational lines after it.
    findstr /c:"error:" "%TEMP%\wireregression_stock.log"
    findstr /c:"Error during build" "%TEMP%\wireregression_stock.log"
    echo --- Wire library actually compiled, from the verbose build log ---
    findstr /i /c:"libraries\\Wire" "%TEMP%\wireregression_stock.log" | findstr /i /c:"Wire.cpp"
    findstr /i /c:"Used library" "%TEMP%\wireregression_stock.log"
    findstr /r /c:"^Sketch uses" "%TEMP%\wireregression_stock.log"
    echo --- full log: %TEMP%\wireregression_stock.log ---
    exit /b %RC%
)

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --build-property "compiler.cpp.extra_flags=%INSTRUMENT%" --library "%WIRE_DIR%" %VENDOR_LIBS% --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --build-property "compiler.cpp.extra_flags=%INSTRUMENT%" --library "%WIRE_DIR%" %VENDOR_LIBS% --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
