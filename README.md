| Supported Target | ESP32-S3 |
| ---------------- | -------- |

# _Drone Firmware_

The is a basic drone driver developed with esp-idf.



## How to use
Connect the usb to the driver board. Then, start 'monitor device'. Meanwhile, Run the python code.

## General File Structure

```
├── components
│   └── drivers
│       ├── CMakeLists.txt
│       ├── driver.c
│       └── driver.h
├── main
│   ├── CMakeLists.txt
│   └── main.c
├── CMakeLists.txt
└── README.md                  This is the file you are currently reading
```
