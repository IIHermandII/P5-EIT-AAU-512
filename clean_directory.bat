@echo off
pushd SDR_testing\data_folder
dir
echo Directory full
popd
if exist SDR_testing\data_folder (
    echo Deleting all files in subfolder data_folder
    del /S /Q SDR_testing\data_folder
) else (
    echo folder does not exist
)
pushd SDR_testing\data_folder
dir
echo Directory empty
popd

