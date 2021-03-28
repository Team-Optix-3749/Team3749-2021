package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Constants;

@SuppressWarnings("PMD.TooManyFields")
public class Drivetrain extends SubsystemBase {
  private WPI_TalonSRX m_leftFrontMotor = new WPI_TalonSRX(Constants.CAN.drive_lf);
  private WPI_VictorSPX m_leftBackMotor = new WPI_VictorSPX(Constants.CAN.drive_lb);
  public SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftFrontMotor, m_leftBackMotor);

  private WPI_TalonSRX m_rightFrontMotor = new WPI_TalonSRX(Constants.CAN.drive_rf);
  private WPI_VictorSPX m_rightBackMotor = new WPI_VictorSPX(Constants.CAN.drive_rb);
  public SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightFrontMotor, m_rightBackMotor);

  public DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // public AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      Constants.Drivetrain.kTrackWidth);
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private final Field2d m_fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem,
      DCMotor.getCIM(2), 8, Constants.Drivetrain.kTrackWidth, Constants.Drivetrain.kWheelRadius, null);

  public Drivetrain() {
    m_gyro.reset();

    m_leftEncoder
        .setDistancePerPulse(2 * Math.PI * Constants.Drivetrain.kWheelRadius / Constants.Drivetrain.kEncoderResolution);
    m_rightEncoder
        .setDistancePerPulse(2 * Math.PI * Constants.Drivetrain.kWheelRadius / Constants.Drivetrain.kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    SmartDashboard.putData("Field", m_fieldSim);
  }

  /**
   * Sets drve speeds
   * @param speeds differential drive wheel speeds
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    m_leftMotors.setVoltage(leftOutput + leftFeedforward);
    m_rightMotors.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drive robot
   * @param xSpeed drive speed
   * @param rot rotation
   */
  public void drive(double xSpeed, double rot) {
    m_drive.setSafetyEnabled(false);

    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  /**
   * update drivetrain odometry
   */
  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * reset drivetrain odometry
   * @param pose Pose2d
   */
  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * get pose
   * @return pose in meters
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * simulation periodic
   * runs in cylces during simulation
   */
  public void simulationPeriodic() {
    m_drivetrainSimulator.setInputs(m_leftMotors.get() * RobotController.getInputVoltage(),
        m_rightMotors.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * curvature drive
   * @param fwd drive along x-axis (positive is forward, negative is backward)
   * @param rot rotation along z-axis
   */
  public void curvatureDrive(double fwd, double rot) {
    m_drive.curvatureDrive(fwd, rot, true);
  }

  /**
   * arcade drive
   * @param fwd drive along x-axis (positive is forward, negative is backward)
   * @param rot rotation along z-axis
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(-fwd * Constants.Drivetrain.kDriveSpeed, rot, true);
  }

  /**
   * classic tank drive
   * @param leftSpeed speed of left side
   * @param rightSpeed speed of right side
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(-leftSpeed, -rightSpeed);
  }

  /**
   * set motor speeds
   * @param leftSpeed speed of left side
   * @param rightSpeed speed of right side
   */
  public void setMotors(double leftSpeed, double rightSpeed) {
    speedLeftMotors(leftSpeed);
    speedRightMotors(rightSpeed);
  }

  /**
   * speed of left side
   * @param speed speed (duh)
   */
  public void speedLeftMotors(double speed) {
    m_leftFrontMotor.set(ControlMode.PercentOutput, -speed);
    m_leftBackMotor.set(ControlMode.PercentOutput, -speed);
  }

  /**
   * speed of right side
   * @param speed speed (duh)
   */
  public void speedRightMotors(double speed) {
    m_rightFrontMotor.set(ControlMode.PercentOutput, speed);
    m_rightBackMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * set motor volts
   * @param leftVolts left side speeds
   * @param rightVolts right side speeds
   */
  public void setOutputVolts(double leftVolts, double rightVolts) {
    m_leftMotors.set(leftVolts / 12);
    m_rightMotors.set(rightVolts / 12);
  }

  /**
   * stop motors (set speed to speed 0)
   */
  public void stopMotors() {
    speedLeftMotors(0);
    speedRightMotors(0);
  }

  /**
   * set max output
   * @param maxOutput max output
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * set get differential drive speeds
   * @return return differential drive speeds
   */
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoderSim.getRate(), m_rightEncoderSim.getRate());
  }

  /**
   * get kinematics
   * @return m_kinematics
   */
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * get feedforward
   * @return m_feedforward
   */
  public SimpleMotorFeedforward getFeedforward() {
    return m_feedforward;
  }

  /**
   * get left PID controller
   * @return m_leftPIDController
   */
  public PIDController getLeftPIDController() {
    return m_leftPIDController;
  }

  /**
   * get rid PID controller
   * @return m_rightPIDController
   */
  public PIDController getRightPIDController() {
    return m_rightPIDController;
  }

  /**
   * get heading
   * @return heading
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Resets gyro
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * reset stuff
   */
  public void reset() {
    m_odometry.resetPosition(new Pose2d(), getHeading());
  }

  /**
   * runs periodically
   */
  public void periodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }
}