idf_component_register(SRCS "protocol-old.cpp" "serial.cpp" "main.cpp" "probe.cpp" "esp32-hal-matrix.c" "esp32-hal-uart.c" "jog.cpp" "WString.cpp" "HardwareSerial.cpp" "motion_control.cpp" "EEPROM.cpp" "serial2socket.cpp" 
                            "grbl_eeprom.cpp" 
                            "protocol.cpp" "system.cpp" "planner.cpp" "gcode.cpp" "report.cpp" 
                            "grbl_eeprom.cpp" "eeprom.cpp"  "stepper.cpp" "esp32-hal-misc.c" "spindle_control.cpp" "coolant_control.cpp" "settings.cpp" "nuts_bolts.cpp" "grbl_limits.cpp" 
                            "HardwareSerial.cpp" "wstring.cpp" "jog.cpp" "esp32-hal-uart.c" "esp32-hal-matrix.c" "probe.cpp" "esp32-hal-cpu.c"
                          
    INCLUDE_DIRS "." "include"
    EMBED_FILES server/error.html server/favicon.ico server/test.css server/test.js server/root.html)