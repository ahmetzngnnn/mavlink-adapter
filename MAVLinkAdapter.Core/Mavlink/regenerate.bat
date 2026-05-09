set PATH=c:\python27;%PATH%

rd /s /q "mavlink"
C:\Python27amd64\python.exe -m pymavlink.tools.mavgen --lang=CS --wire-protocol=2.0 "message_definitions\ardupilotmega.xml"  
copy /y "mavlink\mavlink.cs" "Mavlink.cs"
pause