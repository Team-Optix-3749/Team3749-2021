# Privi Specifications
> Hardware and software specifications for Privi

## Drivetrain
### Programmers: Rebecca & Karthik & Riya
### Setup
- Motors: Four CIM Motors
- Motor Controllers: Two TalonSRX (front), Two VictorSPX (back)
- Encoder: CIM Encoder CIMcoder
- Other Sensors: Kaui Labs AHRS navX-MXP Gyro
### Methods
- arcadeDrive
  - Arcade drive using differential drive
- tankDrive
  - Tank drive using differential drive
- curvatureDrive
  - Curvature drive using differential drive
- driveStraight
  - Drive straight using gyro
- turnAngle
  - Turn to a specific angle
- setMotors
  - Set both sides to percent power
- speedLeftMotors
  - Set left side motors to percent power
- speedRightMotors
  - Set right side motors to percent power
- stopMotors
  - Set all motors to 0
### Commands
- ArcadeDrive
- TankDrive
- CurvatureDrive
- DriveStraight

## Shooter
### Programmers: Rohan & Aadit
### Setup 
- Motors: One NEO motor
- Motor Controllers: One Spark Max
### Methods
- set
  - Set velocity of shooter motor (use built-in NEO encoders)
- stop
  - Set motor to 0
- sendToOrbit
  - Send 12 volts to the motor
- antiJam
  - Make motor go forward and back to unjam power cells
### Commands
- Shoot
- ShootMax
- ShooterAntiJam

# Elevator
### Programmers: Akshay & Safin
### Methods
- elevatorUp
  - Take input percent power
- elevatorDown
  - Take input percent power
- elevatorTop
  - Use encoder to go to top
- elevatorBottom
  - Use encoder to go to bottom
### Commands
- ElevatorUp
- ElevatorDown
- ElevatorTop
- ElevatorBottom

## Intake
### Programmers: Yasaswi & Anthony
### Methods
- up
  - Intake up
- down
  - Intake down
- antiJam
  - Make motors go forward and back to unjam power cells
### Commands
- IntakeUp
- IntakeDown
- IntakeAntiJam
