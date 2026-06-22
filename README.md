# Self-Balancing-Robot-using-ESP32
The developed system employs an MPU6050 inertial measurement unit (IMU) to measure the tilt angle of the robot. A Kalman filter is implemented to fuse accelerometer and gyroscope measurements, reducing sensor noise and providing an accurate estimation of the robot’s orientation. (PID)controller generates the corrective control  to maintain balance. 

4 Control Algorithm Flowchart
The control system of the robot is based on a closed-loop feedback structure that continuously adjusts the motor behavior in order to maintain balance.

Initialization (Startup)
At startup, the system performs the following steps:
1.	Configure motor control pins and PWM generation. 
2.	Initialize communication with the MPU6050 sensor. 
3.	Set up the Kalman filter for angle estimation. 
4.	Calibrate the gyroscope to remove sensor drift. 
5.	Initialize wireless communication (ESP-NOW). 
6.	Load initial control parameters such as PID gains, speed limits, and dead zone. 
 State Estimation
The system continuously estimates the robot’s tilt angle using sensor fusion.
•	Raw sensor data is read from the MPU6050. 
•	The Kalman filter is used to reduce noise and improve stability. 
•	The final estimated angle is corrected.

PID Control Loop
The controller runs at a fixed sampling rate of 5ms.
•	The system calculates the deviation between the desired and actual angle. 
•	A small dead zone is used to ignore very minor oscillations. 
•	The controller computes corrective actions based on proportional, integral, and derivative behavior. 
•	The output is limited to ensure system safety and stability. 
•	The integral term is controlled to avoid excessive accumulation during saturation.
 Motor Control
The control output is converted into motor commands:
•	Direction is determined by the sign of the control signal. 
•	Speed is adjusted proportionally to the controller output. 
•	A minimum threshold is applied to avoid unstable micro-movements. 

Wireless Communication
The system uses ESP-NOW for real-time interaction:
•	The robot receives tuning commands such as PID gains and speed limits. 
•	The system can be adjusted dynamically during operation. 
•	Telemetry data (angle, error, PID output, speed, current parameters) can be transmitted for monitoring and analysis. 

