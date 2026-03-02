#include <Wire.h> //Handles the physical wire between chips
#include <BleMouse.h> //Handles the bluetooth
#include <I2Cdev.h> //I2C Communication
#include <MPU6050_6Axis_MotionApps20.h> //Contains the math for MPU6050 Sensor

BleMouse bleMouse("AirMouse"); //Gives the name to bluetooth signal
MPU6050 mpu;

// Buttons and Leds
#define left_button 4   
#define right_button 5  
#define pause_button 13 
#define status_led 2    // Pin 2 is the built-in blue LED 

//Button State
bool lastLeftState = HIGH;
bool lastRightState = HIGH;
bool lastPauseState = HIGH;
bool isPaused = false;  

//DMP Sensor Variables
bool dmpReady = false;  
uint8_t fifoBuffer[64]; //creates a temporary buffer in ESP's memory

Quaternion q; //4D system to keep the tracking smooth    
VectorFloat gravity; //help differentiate between tilting and sliding via arrow pointing earth's gravity
float ypr[3]; //stores Yaw,Pitch and Roll of sensor   

// Mouse Settings
float sensitivity = 0.12;      
float highSpeedBoost = 0.002;  
float deadzone = 1.5;          
float maxSpeed = 20.0;  

//Smoothing Settings
float smoothingFactor = 0.14; 

float smoothedVelocityX = 0.0;
float smoothedVelocityY = 0.0;
// -------------------------------

float remainderX = 0.0;
float remainderY = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); //Sets the SDA, SCL of pins
  Wire.setClock(400000); 

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 FAILED!");
    while(1); 
  }
  //Calibrating the sensor according to its current placement
  uint8_t devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); 

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("Joystick Mode Ready!");
  } else {
    Serial.println("DMP Init failed");
  }

  pinMode(left_button, INPUT_PULLUP);
  pinMode(right_button, INPUT_PULLUP);
  pinMode(pause_button, INPUT_PULLUP); 
  pinMode(status_led, OUTPUT); // Set the LED pin as an output

  bleMouse.begin(); //Turns on the Bluetooth Radio
  delay(1500); 
}

//Main Loop
void loop() {
  // If not connected to Bluetooth, turn off the LED and halt the loop
  if (!bleMouse.isConnected() || !dmpReady) {
    digitalWrite(status_led, LOW);
    return;
  }

  //Pause Button
  bool currentPause = digitalRead(pause_button);
  if (currentPause != lastPauseState) {
    if (currentPause == LOW) { 
      isPaused = !isPaused;    
      
      if (isPaused) {
        bleMouse.release(MOUSE_LEFT);
        bleMouse.release(MOUSE_RIGHT);
        Serial.println("MOUSE PAUSED");
      } else {
        Serial.println("MOUSE ACTIVE");
      }
    }
    lastPauseState = currentPause;
    delay(15); 
  }

  //Pause Logic
  if (isPaused) {
    digitalWrite(status_led, LOW); // Turn OFF LED when paused
    // Keep emptying the sensor's trash bin so it doesn't crash
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    }
    return; 
  } else {
    digitalWrite(status_led, HIGH); 
  }

  //Left Button
  bool currentLeft = digitalRead(left_button);
  if (currentLeft != lastLeftState) {
    if (currentLeft == LOW) {
      bleMouse.press(MOUSE_LEFT);
    } else {
      bleMouse.release(MOUSE_LEFT);
    }
    lastLeftState = currentLeft;
    delay(10); 
  }

  //Right Button
  bool currentRight = digitalRead(right_button);
  if (currentRight != lastRightState) {
    if (currentRight == LOW) {
      bleMouse.press(MOUSE_RIGHT);
    } else {
      bleMouse.release(MOUSE_RIGHT);
    }
    lastRightState = currentRight;
    delay(10); 
  }

  //Sensor Maths
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float pitch = ypr[1] * 180.0 / M_PI;
    float roll  = ypr[2] * 180.0 / M_PI;

    float targetVelocityX = 0;
    float targetVelocityY = 0;

    // Custom Physics with Optimized Math (Replaced pow() with sqrt())
    if (abs(roll) > deadzone) 
    {
      float activeTilt = abs(roll) - deadzone; 
      targetVelocityX = ((activeTilt * activeTilt) * sensitivity) + ((activeTilt * sqrt(activeTilt)) * highSpeedBoost);
      if (roll < 0) targetVelocityX = -targetVelocityX; 
    }
    
    if (abs(pitch) > deadzone) {
      float activeTilt = abs(pitch) - deadzone;
      targetVelocityY = ((activeTilt * activeTilt) * sensitivity) + ((activeTilt * sqrt(activeTilt)) * highSpeedBoost);
      if (pitch < 0) targetVelocityY = -targetVelocityY; 
    }

    // Limiting the max speed BEFORE smoothing prevents momentum build-up off-screen
    if (targetVelocityX > maxSpeed) targetVelocityX = maxSpeed;
    if (targetVelocityX < -maxSpeed) targetVelocityX = -maxSpeed;
    if (targetVelocityY > maxSpeed) targetVelocityY = maxSpeed;
    if (targetVelocityY < -maxSpeed) targetVelocityY = -maxSpeed;

    // --- APPLY EXPONENTIAL MOVING AVERAGE (EMA) FILTER ---
    smoothedVelocityX = (targetVelocityX * smoothingFactor) + (smoothedVelocityX * (1.0 - smoothingFactor));
    smoothedVelocityY = (targetVelocityY * smoothingFactor) + (smoothedVelocityY * (1.0 - smoothingFactor));

    // Stop the mouse completely if the numbers get microscopically small (prevents eternal drifting)
    if (abs(smoothedVelocityX) < 0.05) smoothedVelocityX = 0;
    if (abs(smoothedVelocityY) < 0.05) smoothedVelocityY = 0;

    remainderX += smoothedVelocityX;
    remainderY += smoothedVelocityY;

    int moveX = (int)remainderX;
    int moveY = (int)remainderY;

    remainderX -= moveX;
    remainderY -= moveY;

    // Move the mouse
    if (moveX != 0 || moveY != 0) {
      bleMouse.move(-moveY, moveX); 
    }
    
    // Removed delay(1) to keep the polling rate locked to the DMP output frequency
  }
}
