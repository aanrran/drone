#include <Wire.h>

#define POLYNOMIAL 0x07
#define SDA_PIN 46
#define SCL_PIN 3
#define I2C_FREQUENCY 50000 // Set I2C frequency to 50 kHz
#define DELAY_BEFORE_READ 2000 // Increased delay in milliseconds before attempting the read
#define LOOP_DELAY 1000 // Delay in milliseconds between each read operation in the loop

struct I2CData {
  uint8_t startByte;
  uint8_t controlWord1;
  uint8_t controlWord2;
  uint8_t controlWord3;
  uint32_t data; // Combined data bytes (e.g., 0x1234ABCD)
};

struct I2CReadControlWords {
  uint8_t startByte;
  uint8_t controlWord0;
  uint8_t controlWord1;
  uint8_t controlWord2;
  uint8_t readStartByte; // Separate start byte for reading
};

uint8_t computeCRC(uint8_t *data, uint8_t length) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ POLYNOMIAL;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void resetI2CBus() {
  // Reinitialize the I2C bus
  Wire.end();
  delay(10); // Short delay to ensure the bus is reset
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);
  Serial.println("I2C bus reset completed.");
}

void i2cRead(I2CReadControlWords readControl) {
  // Reset the I2C bus before read operation
  resetI2CBus();

  // Step 1: Send the read request without a stop condition
  Wire.beginTransmission(readControl.startByte); // Universal start byte
  Wire.write(readControl.controlWord0); // Control word 0
  Wire.write(readControl.controlWord1); // Control word 1
  Wire.write(readControl.controlWord2); // Control word 2
  uint8_t result = Wire.endTransmission(false); // Send without stop condition

  // Print the sent read request
  Serial.print("Sent read request: ");
  Serial.print(readControl.startByte, HEX);
  Serial.print(" ");
  Serial.print(readControl.controlWord0, HEX);
  Serial.print(" ");
  Serial.print(readControl.controlWord1, HEX);
  Serial.print(" ");
  Serial.print(readControl.controlWord2, HEX);
  Serial.println();

  if (result == 0) {
    Serial.println("Read request transmission successful (ACK received).");
  } else {
    Serial.print("Read request transmission failed with error code: ");
    Serial.println(result);
    return;
  }

  // Step 2: Wait for the device to process the request
  delay(DELAY_BEFORE_READ);

  // Step 3: Read the data
  uint8_t bytesRequested = 5;
  uint8_t bytesReceived = Wire.requestFrom(readControl.readStartByte, bytesRequested, true); // Expecting 5 bytes: 4 data bytes, 1 CRC byte
  Serial.print("Bytes requested: ");
  Serial.println(bytesRequested);
  Serial.print("Bytes received: ");
  Serial.println(bytesReceived);

  Serial.print("Received data chain: ");
  uint8_t receivedData[5] = {0};
  int bytesRead = 0;
  while (Wire.available()) {
    receivedData[bytesRead] = Wire.read();
    Serial.print(receivedData[bytesRead], HEX);
    Serial.print(" ");
    bytesRead++;
  }
  Serial.println();

  // Check if we received the expected number of bytes
  if (bytesRead < 5) {
    Serial.print("Expected 5 bytes, but received ");
    Serial.print(bytesRead);
    Serial.println(" bytes.");
    return;
  }

  // Verify the CRC
  uint8_t receivedCRC = receivedData[4];
  uint8_t computedCRC = computeCRC(receivedData, 4); // Compute CRC on the first 4 bytes

  Serial.print("Computed CRC: ");
  Serial.println(computedCRC, HEX);
  Serial.print("Received CRC: ");
  Serial.println(receivedCRC, HEX);

  if (computedCRC == receivedCRC) {
    Serial.println("CRC validation passed.");
  } else {
    Serial.println("CRC validation failed.");
  }

  // Step 4: Send the stop condition
  Wire.endTransmission();
}

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);
  Serial.begin(115200);

  // Initialize the data structure for read operation
  I2CReadControlWords readControl = {
    0x00, // Start byte (universal)
    0xD0, // Control word 0
    0x00, // Control word 1
    0x80, // Control word 2
    0x01  // Start byte for reading
  };

  // Read data once in setup to verify functionality
  i2cRead(readControl);
}

void loop() {
  // Initialize the data structure for read operation
  I2CReadControlWords readControl = {
    0x00, // Start byte (universal)
    0xD0, // Control word 0
    0x00, // Control word 1
    0xA4, // Control word 2
    0x01  // Start byte for reading
  };

  // Read data in the loop with a delay between each read
  i2cRead(readControl);

  // Delay between each read operation
  delay(LOOP_DELAY);
}
