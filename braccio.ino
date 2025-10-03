// Include libraries 
#include <Braccio.h>
#include <Servo.h>
#include <InverseK.h>
#include <math.h>

// Braccio Arm Servos
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

// Sensor HC-SR04 Pins
const int trigPin = 2;
const int echoPin = 4;

// LED Pin
// const int ledPin = 8;

// Variables
long duration;
int distance;
bool objectDetected = false;
const int distanceThreshold = 30;     // Distance threshold for object detection (in cm)
const int baseDistance = 8;           // Base (M1) distance from the sensor 

float a0, a1, a2, a3;

// Link objects for inverse kinematics
Link baseLink, upperarm, forearm, hand;

void setup() {
    // Initialise Braccio arm
    Braccio.begin();

    // Initialise sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(ledPin, OUTPUT);

    // Initialise serial communication
    Serial.begin(9600);

    // Initialise arm in default position
    Braccio.ServoMovement(10, 0, 90, 90, 90, 90, 10);

    // Setup the lengths and rotation limits for each link
    baseLink.init(0, b2a(0.0), b2a(180.0));
    upperarm.init(125, b2a(15.0), b2a(165.0));
    forearm.init(125, b2a(0.0), b2a(180.0));
    hand.init(190, b2a(0.0), b2a(180.0));

    // Attach links to inverse kinematics model
    InverseK.attach(baseLink, upperarm, forearm, hand);
}

void loop() {

    // Rotate base (M1) from 0° to 180°
    for (int angle = 0; angle <= 180; angle += 5) {
       Braccio.ServoMovement(10, angle, 90, 90, 90, 90, 10); // Move base servo
       delay(200); // Delay for smooth movement

        if (detectObject()) {
            Serial.println("Object detected!");
            stopBraccioArm(angle); // Stop arm and calculate IK 
        }   
    }

    // Rotate base (M1) from 180° to 0°
    for (int angle = 180; angle >= 0; angle -= 5) {
        Braccio.ServoMovement(20, angle, 90, 90, 90, 90, 10); // Move base servo
        delay(200); // Delay for smooth movement

        if (detectObject()) {
            Serial.println("Object detected!");
            stopBraccioArm(angle); // Stop arm and calculate IK
        }
    }
}

// Function to detect an object using the HC-SR04 sensor
bool detectObject() {
    // Trigger the sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the echo pulse duration
    duration = pulseIn(echoPin, HIGH);
    // Convert time to distance (in cm)
    distance = duration * 0.034 / 2; 

    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");

    // Return true if an object is within the threshold distance
    return (distance > 0 && distance <= distanceThreshold);
}

// Function to stop Braccio when an object is detected
void stopBraccioArm(int baseAngle) {
    objectDetected = true;
    // digitalWrite(ledPin, HIGH); // Turn on LED

    while (objectDetected) {
      pickUpObject(baseAngle);
  
        // Check if the object is still detected
        if (!detectObject()) {
            Serial.println("No object detected. Resuming motion.");
            objectDetected = false;
            // digitalWrite(ledPin, LOW); // Turn off LED
        }
    }
}

// Function to calculate the angles and pick up object
void pickUpObject(int baseAngle) { 

    // Compute target X, Y position
    distance = distance + baseDistance;
    float X = distance  * 10; // Convert cm to mm
    float Y = 0; 
    float Z = -40;

    Serial.print("Target Coordinates - X: ");
    Serial.print(X);
    Serial.print(" mm, Y: ");
    Serial.print(Y);
    Serial.print(" mm, Z: ");
    Serial.print(Z); 
    Serial.println(" mm");

    // Solve inverse kinematics for calculated (X, Y, Z)
    if (InverseK.solve(X, Y, Z, a0, a1, a2, a3)) {
        Serial.println("Solution found! Moving arm.");
        Serial.println(int(a2b(a3)));

        // Move arm to calculated angles
        Braccio.ServoMovement(20, baseAngle, a2b(a1), a2b(a2), a2b(a3), 90, 10);

        // Close the gripper. Only the M6 servo will move
        Braccio.ServoMovement(20, baseAngle, a2b(a1), a2b(a2), a2b(a3), 90, 73);  

        // Brings the object upwards.
        Braccio.ServoMovement(20,         0,   45, 180,  45,  0, 73);

        // Show the object. Only the M1 servo will moves
        Braccio.ServoMovement(20,         180,  45, 180,   45,   0,  73);

        // Return to the start position.
        Braccio.ServoMovement(20,         0,   90, 180,  180,  90, 73);

        // Open the gripper
        Braccio.ServoMovement(20,         0,   90, 180,  180,  90, 10);
    } else {
            Serial.println("No valid solution found!");
        }

        delay(100); // Small delay for stability
}

// Conversion functions (Braccio degrees <-> Radians)
float b2a(float b) {
    return b / 180.0 * PI - HALF_PI;
}

float a2b(float a) {
    return (a + HALF_PI) * 180 / PI;
}