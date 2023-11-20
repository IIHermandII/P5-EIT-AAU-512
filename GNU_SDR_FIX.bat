@echo off

echo Current directory: %cd%
pushd  ".\SDR_testing\"
set git_path_dll=%cd%\VNX_dps64.dll
echo DLL path: %git_path_dll%

cd /d C:\Users\%USERNAME%\OneDrive\Dokumenter
dir
@REM where we want the file to be 
set doc_path=%cd%
set doc_path_file=%cd%\VNX_dps64.dll

echo le path : %doc_path%
if exist "%doc_path_file%" (
    echo VNX_dps64.dll alredy exist
) else (
    echo VNX_dps64.dll does not exist at %doc_path%
    echo will copy the file from %git_path_dll%
    copy "%git_path_dll%" .
    echo file copyed you can now run Manual phase Control.grc
)
popd
rem dir 
call GNU_TEXTEDDITOR.bat


