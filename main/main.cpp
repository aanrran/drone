#include "Arduino.h"

extern "C"
{
    #include <stdio.h>
}

extern "C"
{
    void app_main(void);
}

void app_main(void)
{
  initArduino();

  // Arduino-like setup()
  Serial.begin(115200);
  while(!Serial){
    ; // wait for serial port to connect
  }

  // Arduino-like loop()
  while(true){
    Serial.println("loop");
  }

}
