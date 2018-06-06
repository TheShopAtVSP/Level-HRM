rem @echo off
nrfjprog --recover
timeout /t 3 >nul
nrfjprog --program v1_02_full.hex --verify 
nrfjprog -r
if NOT ["%errorlevel%"]==["0"] (
    pause
    exit /b %errorlevel%
)

exit
