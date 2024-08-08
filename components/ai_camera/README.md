| Supported Target | ESP32-S3 |
| ---------------- | -------- |

# _Camera Driver_

The is a camera driver for esp32-s3 chip.



## How to use
enter the configuration by type in shell:

```c++
idf.py menuconfig
```

Then, set the following:

* `Serial flasher config → Choose flash mode automatically → Flash SPI mode(DIO)`. 
* `Component config → ESP PSRAM → Support for external, SPI-connected RAM`.
* ` Component config → Camera configuration → Execute camera ISR from IRAM`.

