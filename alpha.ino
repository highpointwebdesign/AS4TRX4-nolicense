//  version 2.6.1

#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU6050.h>
#include <Preferences.h>
#include <math.h> // For sqrt, asin, etc.

// ONBOARD LED
#define LED_PIN         2    // Onboard LED (ESP32)

// Global servo angle constraints
const int servoMin = 15;
const int servoMax = 165;

// The offset from 90 degrees (distance from midpoint).
// e.g., if user wants 75 => offset=15, if user wants 105 => offset=15
int offSetFromMid = 0;

// Create servo objects
Servo frontLeft;
Servo frontRight;
Servo rearLeft;
Servo rearRight;

// Our struct
struct ServoData {
    Servo &servo;
    int pinNumber;
    const char *name;
    int pitchSign;
    int rollSign;
    int rotationAdjustment; // +1 (left side) or -1 (inverted/right side)
    int offset;             // We can store offSetFromMid in here if desired
};

ServoData servoList[] = {
  { frontLeft,  16, "frontLeft",  +1, -1, +1, 0 },
  { frontRight, 17, "frontRight", +1, +1, -1, 0 },
  { rearLeft,    5, "rearLeft",   -1, -1, +1, 0 },
  { rearRight,  18, "rearRight",  -1, +1, -1, 0 }
};

const int NUM_SERVOS = sizeof(servoList) / sizeof(servoList[0]);

// Preferences object
Preferences preferences;

// MPU6050 instance
MPU6050 mpu;

// Angle variables
float angle_pitch = 0.0, angle_roll = 0.0;
float angle_pitch_output = 0.0, angle_roll_output = 0.0;

// Timing
unsigned long stabilizationStartTime = 0;
unsigned long loop_timer = 0;
unsigned long lastSerialUpdate = 0;
bool stabilizationMessagePrinted = false;



// ------------------------------------------
// LED Blink (for calibration, etc.)
// ------------------------------------------
void blinkOnBoardLED() {
  static unsigned long lastBlink = 0;
  static bool ledOn = false;

  if (millis() - lastBlink >= 250) {
    lastBlink = millis();
    ledOn = !ledOn;
    digitalWrite(LED_PIN, (ledOn ? HIGH : LOW));
  }
}

// ------------------------------------------
// Preferences helpers
// ------------------------------------------
// void saveSetting(const char* key, int value) {
//   preferences.putInt(key, value);
//   Serial.printf("Saved %s = %d\n", key, value);
// }
void saveSetting(const char* key, int value) {
  preferences.begin("servo-params", false);

  Serial.printf("DEBUG: Before saving, %s = %d\n", key, preferences.getInt(key, -999));

  // Save the value
  preferences.putInt(key, value);
  Serial.printf("DEBUG: Saved %s = %d\n", key, value);

  // Read it back immediately
  int testRead = preferences.getInt(key, -999);
  Serial.printf("DEBUG: Read back %s = %d (should match saved value)\n", key, testRead);

  // Ensure it's stored in keysList
  String keysList = preferences.getString("keysList", "");
  if (!keysList.equals("") && keysList.indexOf(key) == -1) {
    keysList += ",";
  }
  if (keysList.indexOf(key) == -1) {
    keysList += key;
    preferences.putString("keysList", keysList);
  }

  preferences.end();
}





int loadSetting(const char* key, int defaultValue) {
  preferences.begin("servo-params", true);

  int value = preferences.getInt(key, defaultValue);
  Serial.printf("Loaded %s = %d\n", key, value);
  
  preferences.end();
  
  return value;
}

void listPreferences() {
  preferences.begin("servo-params", true);
  String keysList = preferences.getString("keysList", "");
  
  Serial.println("Stored Preferences:");
  if (keysList.equals("")) {
    Serial.println("No stored preferences.");
  } else {
    int start = 0;
    int end = keysList.indexOf(",");

    while (end != -1) {
      String key = keysList.substring(start, end);
      int value = preferences.getInt(key.c_str(), -999); 
      Serial.printf("Key: %s, Value: %d\n", key.c_str(), value);
      start = end + 1;
      end = keysList.indexOf(",", start);
    }

    // Print last key
    String key = keysList.substring(start);
    int value = preferences.getInt(key.c_str(), -999);
    Serial.printf("Key: %s, Value: %d\n", key.c_str(), value);
  }
  preferences.end();
}



// ------------------------------------------
// Servo move helper
// ------------------------------------------
void moveServo(Servo &servoDesignation, const char* servoName, int degree) {
  int safeAngle = constrain(degree, servoMin, servoMax);
  servoDesignation.write(safeAngle);
  Serial.printf("Updated %s to %d degrees\n", servoName, safeAngle);
}

// ------------------------------------------
// MPU init
// ------------------------------------------
void initMPU() {
  stabilizationStartTime = millis(); // Set stabilization start time
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully.");
  } else {
    Serial.println("MPU6050 connection failed. Check wiring!");
    while (1);
  }
}

// ------------------------------------------
// Preferences init
// ------------------------------------------
// void initPreferences() {
//   preferences.begin("servo-params", false);
//   // Load the offset from 90 that we saved previously
//   offSetFromMid = loadSetting("offSetFromMid", 0);
// }
void initPreferences() {
  preferences.begin("servo-params", true);
  
  // Attempt to load the stored offset
  offSetFromMid = preferences.getInt("offSetFromMid", -999);
  
  Serial.printf("DEBUG: Loaded offSetFromMid = %d\n", offSetFromMid);
  
  preferences.end();
}




// ------------------------------------------
// Attach all servos
// ------------------------------------------
void initAttachServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoList[i].servo.attach(servoList[i].pinNumber);
    servoList[i].offset = offSetFromMid;  // Apply loaded offset

    // Move each servo to the correct position based on saved offset
    int finalAngle = 90 + (servoList[i].rotationAdjustment * servoList[i].offset);
    finalAngle = constrain(finalAngle, servoMin, servoMax);
    
    servoList[i].servo.write(finalAngle);
    Serial.printf("DEBUG: %s initialized to %d degrees (offset %d)\n",
                  servoList[i].name, finalAngle, servoList[i].offset);
  }
}



// ------------------------------------------
// Calibration with LED blinking
// ------------------------------------------
void calibrateGyroWithBlink() {
  Serial.println("Calibrating gyro... (LED blinking)");
  angle_pitch = 0.0;
  angle_roll  = 0.0;

  for (int i = 0; i < 2000; i++) {
    blinkOnBoardLED();

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    angle_pitch += gx;
    angle_roll  += gy;

    delay(3);
  }

  angle_pitch /= 2000.0;
  angle_roll  /= 2000.0;

  Serial.println("Gyro calibrated.");
  // LED solid ON
  digitalWrite(LED_PIN, HIGH);
}

// -----------------
// Handle Serial Commands
// -----------------
void handleSerialCommands() {
  if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');  // Read full line
    input.trim();  // Remove whitespace
    
    Serial.printf("DEBUG: Received input: [%s]\n", input.c_str());  // Print raw input

    if (input.startsWith("rideheight")) {
      Serial.println("DEBUG: Command recognized as 'rideheight'");

      // 1) Parse user input
      int rawAngle = input.substring(input.indexOf(' ') + 1).toInt();

      // 2) Validate rawAngle in range [servoMin..servoMax]
      if (rawAngle >= servoMin && rawAngle <= servoMax) {
        
        // 3) Calculate new offset (Difference from 90)
        int offsetVal = rawAngle - 90;  // Can be positive or negative!

        // 4) Constrain offset within valid range
        offsetVal = constrain(offsetVal, servoMin - 90, servoMax - 90);  

        // 5) Save new offset to preferences
        saveSetting("offSetFromMid", offsetVal);
        offSetFromMid = offsetVal; // Update global variable


        // 6) Update each servo's offset in the struct
        for (int i = 0; i < NUM_SERVOS; i++) {
            servoList[i].offset = offsetVal;
        }

        // 7) Move Servos *immediately* to the correct position
        for (int i = 0; i < NUM_SERVOS; i++) {
          // Use servoList[i].rotationAdjustment instead of just rotationAdjustment
          int finalAngle;
          if (servoList[i].rotationAdjustment == +1) {
            finalAngle = 90 - offsetVal;
          } else {
            finalAngle = 90 + offsetVal;
          }

          finalAngle = constrain(finalAngle, servoMin, servoMax);
          moveServo(servoList[i].servo, servoList[i].name, finalAngle);
        }


        // 8) Reset the stabilization period
        stabilizationStartTime = millis();
        Serial.printf("Offset updated to %d. Servos moved immediately.\n", offsetVal);
        printMenu();
      } 
      else {
        Serial.println("Invalid position. Please enter a value between 15 and 165.");
      }
    } else if (input.startsWith("listpreferences")){
      listPreferences();
    } else if (input.startsWith("resetgyro")){
      calibrateGyroWithBlink();

    } else if (input.startsWith("min")){
      minChassis();
    } else if (input.startsWith("max")){
      maxChassis();
    } else if (input.startsWith("up")){
      raiseChassis();
    } else if (input.startsWith("down")){
      lowerChassis();
    }
    else {
      Serial.println("Unknown command. Use 'rideheight <value>'.");
      printMenu();
    }
  }
}

void raiseChassis(){
  Serial.println("DEBUG: Command 'up' corresponds to 'raise chassis'");

  // Get the current offset
  int currOffSet = offSetFromMid;

  // Increase the offset by 10 degrees
  int offsetVal = currOffSet + 10;

  // Constrain offset within valid range
  offsetVal = constrain(offsetVal, servoMin - 90, servoMax - 90);

  // Save updated offset
  saveSetting("offSetFromMid", offsetVal);
  offSetFromMid = offsetVal; // Update global variable

  // Apply new offset to all servos
  for (int i = 0; i < NUM_SERVOS; i++) {
      servoList[i].offset = offsetVal;
  }

  // Move servos to their new position
  for (int i = 0; i < NUM_SERVOS; i++) {
    int finalAngle = 90 + (servoList[i].rotationAdjustment * offsetVal);

    // Ensure servo position is within limits
    finalAngle = constrain(finalAngle, servoMin, servoMax);
    
    moveServo(servoList[i].servo, servoList[i].name, finalAngle);
  }
}


void lowerChassis() {
  Serial.println("DEBUG: Command 'down' corresponds to 'lower chassis'");

  // Get the current offset
  int currOffSet = offSetFromMid;

  // Decrease the offset by 10 degrees
  int offsetVal = currOffSet - 10;

  // Constrain offset within valid range
  offsetVal = constrain(offsetVal, servoMin - 90, servoMax - 90);

  // Save updated offset
  saveSetting("offSetFromMid", offsetVal);
  offSetFromMid = offsetVal; // Update global variable

  // Apply new offset to all servos
  for (int i = 0; i < NUM_SERVOS; i++) {
      servoList[i].offset = offsetVal;
  }

  // Move servos to their new position
  for (int i = 0; i < NUM_SERVOS; i++) {
    int finalAngle = 90 + (servoList[i].rotationAdjustment * offsetVal);

    // Ensure servo position is within limits
    finalAngle = constrain(finalAngle, servoMin, servoMax);
    
    moveServo(servoList[i].servo, servoList[i].name, finalAngle);
  }
}


void minChassis(){
  Serial.println("DEBUG: Command 'min' corresponds to 'chassis in the lowest position'");

  // Set the offset to the lowest ride height possible
  int offsetVal = servoMin - 90;

  // Save updated offset
  saveSetting("offSetFromMid", offsetVal);
  offSetFromMid = offsetVal; // Update global variable

  // Apply new offset to all servos
  for (int i = 0; i < NUM_SERVOS; i++) {
      servoList[i].offset = offsetVal;
  }

  // Move servos to their new position (force lowest position)
  for (int i = 0; i < NUM_SERVOS; i++) {
    int finalAngle = servoMin;  // Directly move servos to their lowest possible position

    moveServo(servoList[i].servo, servoList[i].name, finalAngle);
  }
}


void maxChassis(){
  Serial.println("DEBUG: Command 'max' corresponds to 'chassis in the highest position'");

  // Set the offset to the highest ride height possible
  int offsetVal = servoMax - 90;

  // Save updated offset
  saveSetting("offSetFromMid", offsetVal);
  offSetFromMid = offsetVal; // Update global variable

  // Apply new offset to all servos
  for (int i = 0; i < NUM_SERVOS; i++) {
      servoList[i].offset = offsetVal;
  }

  // Move servos to their new position (force highest position)
  for (int i = 0; i < NUM_SERVOS; i++) {
    int finalAngle = servoMax;  // Directly move servos to their highest possible position

    moveServo(servoList[i].servo, servoList[i].name, finalAngle);
  }
}



// ------------------------------------------
// Calculate Final Servo Angles
// ------------------------------------------
void calculateFinalServoAngle(float angle_pitch, float angle_roll) {
  for (int i = 0; i < NUM_SERVOS; i++) {
  // 1) combine pitch & roll
  float servoInput = (servoList[i].pitchSign * angle_pitch)
                   + (servoList[i].rollSign  * angle_roll);

  // 2) outMin/outMax
  int outMin = (servoList[i].rotationAdjustment == +1) ? servoMin : servoMax;
  int outMax = (servoList[i].rotationAdjustment == +1) ? servoMax : servoMin;

  // 3) map
  int mappedVal = map((int)servoInput, -45, 45, outMin, outMax);

  // 4) apply offset
  //    remove extra declarations of finalAngle
  //    just declare it once
  int offsetSign = (servoList[i].rotationAdjustment == +1) ? -1 : +1;
  int finalAngle = mappedVal + (offsetSign * servoList[i].offset);

  finalAngle = constrain(finalAngle, servoMin, servoMax);
  servoList[i].servo.write(finalAngle);
}

}

// ------------------------------------------
// (Optional) Serial Output for debugging
// ------------------------------------------
void serialOutput() {
  if (millis() - lastSerialUpdate >= 2000) {
    Serial.print("Pitch: ");
    Serial.print((int)angle_pitch_output);
    if (angle_pitch_output > 2) {
      Serial.print(" (tilted forward)");
    } else if (angle_pitch_output < -2) {
      Serial.print(" (tilted backward)");
    } else {
      Serial.print(" (level)");
    }

    Serial.print(" | Roll: ");
    Serial.print((int)angle_roll_output);
    if (angle_roll_output > 2) {
      Serial.println(" (tilted right)");
    } else if (angle_roll_output < -2) {
      Serial.println(" (tilted left)");
    } else {
      Serial.println(" (level)");
    }
    lastSerialUpdate = millis();
  }
}

// ------------------------------------------
// PRINT MENU
// ------------------------------------------
void printMenu() {
  Serial.println("============================");
  Serial.println("|Menu Options:             |");
  Serial.println("|  rideheight <value>      |");
  Serial.println("|  listpreferences         |");
  Serial.println("|  resetgyro               |");
  Serial.println("|  min                     |");
  Serial.println("|  max                     |");
  Serial.println("|  up                      |");
  Serial.println("|  down                    |");
  Serial.println("============================");
  Serial.println();
  listPreferences();

}


// ------------------------------------------
// Update Pitch & Roll from MPU
// ------------------------------------------
void updateAngles(int16_t ax, int16_t ay, int16_t az,
                  int16_t gx, int16_t gy, int16_t gz)
{
  // Gyro integration
  angle_pitch += gx * 0.0000611f;
  angle_roll  += gy * 0.0000611f;

  // Acc angles
  float acc_total_vector = sqrtf(ax*ax + ay*ay + az*az);
  if (acc_total_vector == 0) acc_total_vector = 1;
  float angle_pitch_acc = asinf((float)ay / acc_total_vector) * 57.296f;
  float angle_roll_acc  = asinf((float)ax / acc_total_vector) * -57.296f;

  // Complementary filter
  angle_pitch = angle_pitch * 0.98f + angle_pitch_acc * 0.02f;
  angle_roll  = angle_roll  * 0.98f + angle_roll_acc  * 0.02f;

  // Smooth
  angle_pitch_output = angle_pitch_output * 0.9f + angle_pitch * 0.1f;
  angle_roll_output  = angle_roll_output  * 0.9f + angle_roll  * 0.1f;
}

// ------------------------------------------
// setup()
// ------------------------------------------
void setup() {
    delay(500); // Small delay to let Serial stabilize

  // Clear any junk characters
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println();
  Serial.println("\nESP32 Booting...");


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin();
  Serial.begin(115200);

  initMPU();
  initPreferences();
  initAttachServos();

  // Blink LED while calibrating
  calibrateGyroWithBlink();

  Serial.println("Setup complete.");
  Serial.printf("Ride Height set to %d.\n", offSetFromMid);
  printMenu();
  
  // listPreferences();

}

// ------------------------------------------
// loop()
// ------------------------------------------
void loop() {
  handleSerialCommands();

  // read sensor
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // update angles
  updateAngles(ax, ay, az, gx, gy, gz);

  // if >2s from startup, apply balancing logic
  if (millis() - stabilizationStartTime > 2000) {
    calculateFinalServoAngle(angle_pitch_output, angle_roll_output);
  } else {
    if (!stabilizationMessagePrinted) {  // Print only once
      Serial.println("Stabilization period active: Balancing logic is disabled.");
      stabilizationMessagePrinted = true;  // Set flag to true so it doesn't print again
    }
  }

  // optionally print angles
  // serialOutput();

  // ~250 Hz loop
  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}
