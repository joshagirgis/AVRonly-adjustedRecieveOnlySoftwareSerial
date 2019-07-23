# AVRonly-adjustedRecieveOnlySoftwareSerial
 No Arduino Dependencies,
 Adjust your Port Settings in the .cpp
 
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
