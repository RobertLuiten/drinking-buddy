#include <Servo.h>

Servo servoX;
Servo servoY;

const int minPos = 75;
const int maxPos = 105;
const int yStep = 5;

// Increase this for slower/smoother motion, decrease for faster
const int stepDelay = 30; 

void setup() {
  servoX.attach(9);
  servoY.attach(10);
  
  // Smoothly move to start position
  servoX.write(minPos);
  servoY.write(minPos);
  delay(1000);
}

void loop() {
  for (int y = minPos; y <= maxPos; y += yStep) {
    servoY.write(y);
    delay(200); // Give Y time to settle

    // Determine direction based on current row (Snake pattern)
    // If row index is even, go left to right. If odd, go right to left.
    if (((y - minPos) / yStep) % 2 == 0) {
      // Forward Sweep (75 -> 105)
      for (float pos = minPos; pos <= maxPos; pos += 0.5) {
        servoX.writeMicroseconds(map(pos, 0, 180, 544, 2400));
        delay(stepDelay);
      }
    } else {
      // Backward Sweep (105 -> 75)
      for (float pos = maxPos; pos >= minPos; pos -= 0.5) {
        servoX.writeMicroseconds(map(pos, 0, 180, 544, 2400));
        delay(stepDelay);
      }
    }
  }
  
  // Reset sequence
  delay(1000);
  servoY.write(minPos);
  delay(500);
}
