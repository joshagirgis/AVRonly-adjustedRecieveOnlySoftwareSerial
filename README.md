# AVRonly-adjustedRecieveOnlySoftwareSerial
 No Arduino Dependencies,
 Adjust your Port Settings in the .cpp.
 The pin must be an interrupt pin
 
## Usage:
```
  #include <ReceiveOnlySoftwareSerial.h>
   ReceiveOnlySoftwareSerial gpsSer;  // RX 
   ```
   
   ```
if (gpsSer.available()) {
    char c = gpsSer.read();
}
   ```
