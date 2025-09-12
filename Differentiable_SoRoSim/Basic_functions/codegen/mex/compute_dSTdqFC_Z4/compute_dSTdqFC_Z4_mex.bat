@echo off
call setEnv.bat
"C:\Program Files\MATLAB\R2024a\toolbox\shared\coder\ninja\win64\ninja.exe" -t compdb cc cxx cudac > compile_commands.json
"C:\Program Files\MATLAB\R2024a\toolbox\shared\coder\ninja\win64\ninja.exe" -v %*
