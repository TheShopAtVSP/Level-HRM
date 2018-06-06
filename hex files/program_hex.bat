@echo off

:getfile
echo Enter File Name:
set /p FILE=""
if EXIST %FILE%.hex (
    echo Found File: %FILE%.hex
    echo(
) else (
    echo File %FILE%.hex Not Found
    echo(
    goto getfile
)

:loop
set STARTTIME=%TIME%
nrfjprog --recover 
timeout /t 3 >nul

nrfjprog --program %FILE%.hex --verify -r --clockspeed 2000
if NOT ["%errorlevel%"]==["0"] (
    echo Programming FAILED
    pause
    exit /b %errorlevel%
)

echo(
echo(
set ENDTIME=%TIME%
set /A STARTTIME=(1%STARTTIME:~0,2%-100)*360000 + (1%STARTTIME:~3,2%-100)*6000 + (1%STARTTIME:~6,2%-100)*100 + (1%STARTTIME:~9,2%-100)
set /A ENDTIME=(1%ENDTIME:~0,2%-100)*360000 + (1%ENDTIME:~3,2%-100)*6000 + (1%ENDTIME:~6,2%-100)*100 + (1%ENDTIME:~9,2%-100)
set /A DELTA=ENDTIME-STARTTIME
set /A s=DELTA/100
set /A ds=DELTA-s*100
set /A ds/=10
rem echo delta=%DELTA% s=%s%  ds=%ds%
echo Device programmed successfully in %s%.%ds% s.
echo Press any key to program the next device.

pause >nul
echo(
goto loop
exit