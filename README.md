As of now, this code should work given these assumptions.
1. The drive motor is a NEO
2. The steering motor is a PG71
3. The drive motor controller is a Spark Max
4. The steering motor controller is a Talon SRX
5. The absolute encoder is tethered to the steering Talon SRX
6. The Spark Max returns velocity as RPM
7. All measurements are in meters

As of now, there is still information needed before use
1. Absolute encoder resolution
2. Wheelbase footprint, NOT frame dimentions
3. CAN ids for all motor controllers
4. PID controller tuning

Code uses template made by WPILib with changes for the motor controllers that I hope to use. 