As of now, this code should work given these assumptions.
1. The drive motor is a NEO
2. The steering motor is a PG71
3. The drive motor controller is a Spark Max
4. The steering motor controller is a Talon SRX
5. All measurements are metric
6. All angles are measured in radians

As of now, there is still information needed before use
1. CAN ids for all motor controllers
2. PID controller tuning

Code uses template made by WPILib with changes for the motor controllers that I hope to use. Additionally, there are major changes to make it command-based.
