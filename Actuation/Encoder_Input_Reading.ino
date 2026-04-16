#include <SoftwareWire.h>

// --- PIN CONFIGURATION ---
// User requested: SDA on A1, SCL on A0
#define SOFT_SDA A5 
#define SOFT_SCL A4

// --- I2C ADDRESSES ---
#define MUX_ADDR 0x70       // PCA9548A Multiplexer
#define AS5600_ADDR 0x36    // AS5600 Encoder
#define RAW_ANGLE_REG 0x0E  // Register for 12-bit raw angle

// Initialize SoftwareWire on the requested pins
SoftwareWire myWire(SOFT_SDA, SOFT_SCL);

void setup() {
  Serial.begin(115200);
  myWire.begin();
  
  Serial.println("--- PCA9548A + AS5600 Software I2C Test ---");
  Serial.println("Pins: SDA=A1, SCL=A0");
  Serial.println("Scanning channels...");
}

void loop() {
  for (uint8_t i = 0; i < 3; i++) {
    selectMuxChannel(i);
    int angle = readAS5600();
    
    Serial.print("CH");
    Serial.print(i);
    Serial.print(": ");
    if (angle == -1) {
      Serial.print("NOT FOUND  ");
    } else {
      Serial.print(angle);
      Serial.print(" pts  ");
    }
  }
  Serial.println(); // New line for next set of readings
  delay(200);       // Readable update rate
}

// --- HELPER FUNCTIONS ---

void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  
  myWire.beginTransmission(MUX_ADDR);
  myWire.write(1 << channel); // Bitmask to enable specific channel
  myWire.endTransmission();
}

int readAS5600() {
  myWire.beginTransmission(AS5600_ADDR);
  myWire.write(RAW_ANGLE_REG);
  if (myWire.endTransmission() != 0) return -1; // Device not responding

  myWire.requestFrom(AS5600_ADDR, 2);
  if (myWire.available() == 2) {
    uint8_t highByte = myWire.read();
    uint8_t lowByte = myWire.read();
    return (highByte << 8) | lowByte; // Combine into 12-bit value (0-4095)
  }
  
  return -1;
}
