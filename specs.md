# Privi Specifications
> Hardware and software specifications for Privi

## Drivetrain
### Programmers: [REDACTED]
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
### Programmers: [REDACTED]
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

## Intake
### Programmers: [REDACTED]
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
