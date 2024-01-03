@echo off
setlocal enabledelayedexpansion

set "command_file_prefix=split_"
set "mingwenv_script=..\..\..\mingwenv.cmd"

for %%F in (%command_file_prefix%*.txt) do (
    set "current_dir=!CD!"
    start "MinGW Environment" cmd /c "cd /d !current_dir! && %mingwenv_script% -mingw32 -c "%%F"
)

goto :eof

:run_commands
set "command_file=%1"

:: Add your custom logic here to process each command file
:: For example, you can read the content of the file and execute commands
:: The placeholder command below simply echoes the file name for demonstration purposes
echo Processing commands from file: %command_file%

:: Replace the above echo line with your actual command execution logic
:: Example: 
:: for /f "tokens=*" %%A in (%command_file%) do (
::     echo Running command: %%A
::     call %%A
:: )

goto :eof