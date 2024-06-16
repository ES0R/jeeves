
# ESP32-LED

## Commands
For programming ESP
```
. 'C:\Users\emilo\ws\Projects\esp-idf\export.ps1'
idf.py create-project esp32-led
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash
idf.py monitor
```


## TODO
- [ ] Interrupt instead of polling
- [ ] GPIO stuff for LED
- [ ] OTA for reconfiguration

