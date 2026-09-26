@echo off
setlocal enabledelayedexpansion

rem Host tests for the MKR Zero firmware logic, built with MSVC:
rem   switch_tests      lib\SwitchFunctions.cpp
rem   imu_tests         lib\IMUFunctions.cpp lifecycle and fault accounting
rem   can_probe_tests   lib\CANSniffFunctions.cpp map probe (MCP2515 model)
rem
rem   RunTests.cmd        build and run every suite
rem
rem Exit code is the total number of failing checks, so this is usable as a gate.
rem
rem There is a Makefile beside this file for g++ / clang++ - use that on the
rem Orin Nano or any Linux box. This script exists because the Windows machine
rem this firmware is normally built from has no g++ at all: arduino-cli ships
rem only arm-none-eabi, which cross-compiles and cannot run what it produces.
rem Tests that cannot be executed where the code is written do not get run.
rem
rem THIS DIRECTORY IS FIRST ON THE INCLUDE PATH ON PURPOSE. The modules under
rem test include <Arduino.h>, <Wire.h>, <SPI.h>, <CAN.h> and <SdFat.h> with angle
rem brackets, so the stand-ins here shadow the real ones and every module
rem compiles UNMODIFIED - no #ifdef, no test-only build of the firmware, and no
rem second copy of the logic to drift out of sync.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

set "VCVARS="
for %%E in (Community Professional Enterprise BuildTools) do (
    for %%V in (18 2022 2019) do (
        if not defined VCVARS (
            if exist "%ProgramFiles%\Microsoft Visual Studio\%%V\%%E\VC\Auxiliary\Build\vcvars64.bat" (
                set "VCVARS=%ProgramFiles%\Microsoft Visual Studio\%%V\%%E\VC\Auxiliary\Build\vcvars64.bat"
            )
            if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\%%V\%%E\VC\Auxiliary\Build\vcvars64.bat" (
                set "VCVARS=%ProgramFiles(x86)%\Microsoft Visual Studio\%%V\%%E\VC\Auxiliary\Build\vcvars64.bat"
            )
        )
    )
)

if not defined VCVARS (
    echo No MSVC installation found.
    echo Install "Desktop development with C++", or use the Makefile with g++.
    exit /b 1
)

rem vcvars prints a harmless complaint when vswhere is absent; it still works.
call "%VCVARS%" >nul 2>&1

rem NOT named LIB. That is MSVC's own library search path, exported by vcvars
rem above, and overwriting it makes the compile fail at link with
rem "cannot open file 'LIBCMT.lib'" - which reads as a broken toolchain
rem installation rather than as this script standing on it.
for %%I in ("%~dp0.") do set "TESTS=%%~fI"
for %%I in ("%TESTS%\..\..\lib") do set "LIBDIR=%%~fI"

cd /d "%TESTS%"

set "CL_FLAGS=/nologo /std:c++14 /W4 /EHsc /I"%TESTS%" /I"%LIBDIR%""
set "RC=0"

cl %CL_FLAGS% arduino_stub.cpp switch_tests.cpp "%LIBDIR%\SwitchFunctions.cpp" /Fe:switch_tests.exe
if errorlevel 1 goto :buildfail

cl %CL_FLAGS% arduino_stub.cpp imu_tests.cpp "%LIBDIR%\IMUFunctions.cpp" /Fe:imu_tests.exe
if errorlevel 1 goto :buildfail

cl %CL_FLAGS% arduino_stub.cpp bno_init_tests.cpp "%LIBDIR%\BNO055Init.cpp" /Fe:bno_init_tests.exe
if errorlevel 1 goto :buildfail

cl %CL_FLAGS% arduino_stub.cpp mcp2515_model.cpp can_probe_tests.cpp ^
   "%LIBDIR%\CANSniffFunctions.cpp" "%LIBDIR%\CANMap.cpp" "%LIBDIR%\VehicleSignals.cpp" ^
   /Fe:can_probe_tests.exe
if errorlevel 1 goto :buildfail

del /q *.obj 2>nul
echo.

rem Every suite runs even after one fails, so a single run reports everything.
for %%T in (switch_tests imu_tests can_probe_tests bno_init_tests) do (
    .\%%T.exe
    set /a RC+=!ERRORLEVEL!
)
exit /b %RC%

:buildfail
del /q *.obj 2>nul
echo Build failed.
exit /b 1
