# 🤖 Robotic Arm System

The project aimed to develop a robotic arm system for automated object manipulation, achieving object detection, implementing a reliable gripping mechanism and enabling object repositioning. The system uses a Braccio Tinkerkit robotic arm controlled by an Arduino microcontroller with an HC-SR04 ultrasonic sensor for real-time object detection.

## 📥 Installation & Usage

### Prerequisites
- Arduino IDE installed on your computer
- Braccio Tinkerkit robotic arm
- Arduino board 
- HC-SR04 ultrasonic sensor

### Required Libraries
Install the following libraries through Arduino IDE Library Manager:
- `Braccio` - For Braccio arm control
- `Servo` - For servo motor control
- `InverseK` - For inverse kinematics calculations (available on GitHub)

### Hardware Setup
1. Assemble the Braccio robotic arm according to manufacturer instructions
2. Connect the HC-SR04 ultrasonic sensor
3. Attach the sensor to the robot's base for optimal object detection


### Software Installation
1. Download or clone the `braccio.ino` file
2. Open the file in Arduino IDE
3. Select your Arduino board: `Tools > Board > [Your Arduino Model]`
4. Select the correct COM port: `Tools > Port > [Your Port]`

### How to Use
1. Power on the Braccio arm and ensure all connections are secure
2. The arm will initialise to its default position (all servos at 90°, gripper open)
3. The base will begin rotating from 0° to 180° and back, scanning for objects
4. Place an object within 30 cm of the sensor
5. When detected, the arm will:
   - Stop scanning
   - Calculate required angles using inverse kinematics
   - Move to the object position
   - Close gripper to grasp the object
   - Lift and rotate to display the object
   - Move to drop-off position
   - Release the object
6. The system will resume scanning after release

## ⚙️ System Architecture

The Braccio Tinkerkit robotic arm system consists of four main components:


<img width="616" height="444" alt="Screenshot 2025-10-06 at 14 03 17" src="https://github.com/user-attachments/assets/6750231e-6fd4-4247-b90a-471d07bf15a5" />


### Servo Motors

| Motor | Function | Range | What it does |
|-------|----------|-------|--------------|
| M1 (Base) | Rotates the whole arm | 0° – 180° | Moves the arm left & right |
| M2 (Shoulder) | Moves the upper arm | 15° – 165° | Moves up & down |
| M3 (Elbow) | Bends the elbow | 0° – 180° | Extends or retracts the arm |
| M4 (Wrist Vertical) | Moves wrist up/down | 0° – 180° | Adjusts wrist angle |
| M5 (Wrist Rotation) | Rotates the wrist | 0° – 180° | Rotates objects |
| M6 (Gripper) | Opens & closes gripper | 10° – 73° | Grasps and releases objects |

### Motor Shield
Simplifies motor control by providing motor drivers and power management, ensuring stable power delivery to servo motors.

### Ultrasonic Distance Sensor (HC-SR04)
Detects obstacles by emitting an ultrasonic pulse and measuring the time it takes for the echo to return.

**Wiring Configuration:**
- VCC → 5V
- GND → GND
- Trig → Digital Pin 2
- Echo → Digital Pin 4

### Arduino Microcontroller & IDE
Serves as the central unit, managing communication between components, generating PWM signals for servo motor control and processing sensor data.

## 🔄 System Operation

1. Motor shield supplies power to servo motors, while Arduino operates separately
2. Ultrasonic sensor measures distances and sends data to Arduino
3. Arduino processes sensor data and decides movement
4. Servo motors adjust angles based on PWM signals from Arduino
5. System continuously repeats this process, ensuring smooth and autonomous operation

## 📡 Sensor Integration & Testing

The HC-SR04 sensor was tested by measuring distances of 5 cm, 10 cm, 15 cm, 20 cm, 25 cm and 30 cm, with each measurement repeated five times. The readings remained consistent with a small error margin of ±3mm. While accuracy slightly decreases with distance, the sensor performs reliably within its 5 cm to 30 cm range.

<img width="410" height="327" alt="Screenshot 2025-10-06 at 14 03 50" src="https://github.com/user-attachments/assets/d82c0209-71f6-43ea-871d-db3069627973" />


**Distance Measurement Process:**
1. Trig pin is set "high" for 10µs, triggering 8 ultrasonic pulses at 40kHz
2. Echo pin is set "high", ready to detect returning waves
3. If waves hit an object and reflect, Echo pin registers signal and switches "low"
4. Time duration allows for distance calculation using speed of sound

<img width="814" height="321" alt="Screenshot 2025-10-06 at 14 04 26" src="https://github.com/user-attachments/assets/bbe74ded-7fac-4437-b7a3-f08615662564" />

## 🧮 Inverse Kinematics

The system uses inverse kinematics to work backward from the object's position to calculate the necessary joint angles. By estimating the angles at which motors should move, the arm can be directed to the target position.

**Key formulas derived:**
- Cosine rule is fundamental to inverse kinematics calculations
- InverseK function computes angle configurations for arm to reach and grasp objects
- System operates in two-dimensional plane (X and Z axes, Y axis fixed at zero)

## 🎮 Path Planning Algorithm

### 1. Scanning
- Move base from 0° to 180° and back while scanning for objects
- If object detected, scanning stops and inverse kinematics computes arm movement

### 2. Object Handling
- Move arm to detected object position with calculated angles
- Grip the object
- Lift and transport object to predefined drop location
- Open gripper to release object

### 3. Returning to Scanning
- Verify object is no longer detected
- Resume scanning from starting position

<img width="599" height="596" alt="Screenshot 2025-10-06 at 14 05 17" src="https://github.com/user-attachments/assets/94953c40-34eb-4ceb-ac68-dd710ad92861" />
  

## 📊 Performance Metrics

| Metric | Values/Observations |
|--------|---------------------|
| Pick-and-place success rate | 7/10 successful attempts (70%) |
| Gripper success rate | 70% |
| Sensor distance and accuracy | ±5 cm for distances < 30 cm; less accurate beyond 30 cm |
| Time for operation | 14.7 seconds |
| Maximum reach | ~32 cm (from base) |
| Payload capacity | 150g (fully extended), 400g (closer to base) |

<img width="599" height="596" alt="Screenshot 2025-10-06 at 14 06 21" src="https://github.com/user-attachments/assets/2962ef3b-64cf-4c29-aaf3-d1f072347bdf" />

## ✅ Task Objectives Achievement

| Objective | Achieved? |
|-----------|-----------|
| Object detection | ✓ |
| Alignment with object | ✓ |
| Secure grip | ✓ |
| Movement from one point to another | ✓ |
| Safe release | ✓ |

## 🔧 Key Learnings

- Importance of systematic assembly and component verification to prevent delays
- Translating mathematical concepts (inverse kinematics) into functional code requires dedicated research and iterative testing
- Sensor calibration is critical for accurate distance measurements and system reliability
- Pin assignment conflicts can prevent proper system operation and require careful planning
- Teamwork and persistence are essential for overcoming hardware and software challenges
- Real-world testing reveals limitations not apparent in theoretical planning (e.g., gripper alignment with larger objects)
- Integration of multiple systems (sensors, motors, algorithms) requires careful coordination and debugging
- Understanding kinematic equations and coordinate constraints is fundamental for precise robotic control

## 💡 Future Improvements

- Upgrade to higher-quality motors for smoother and more precise movements
- Reinforce joints with stronger screws to improve durability
- Implement dynamic release location adjustment based on real-time calculations
- Add additional sensor to detect object height and size for improved grip alignment
- Enhance system to handle larger and taller objects more effectively
