rem Saved in D:\Temp\WriteText.bat
@echo off
cd C:\Users\%username%\AppData\Roaming\.gnuradio\
rem dir
if exist config.conf (
    echo config.conf does exist
) else (
    echo the config.conf will be createt
    echo [grc]>> config.conf
    echo editor = C:\Users\%username%\AppData\Local\Programs\Microsoft VS Code\Code.exe>> config.conf
)

