package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

/**
 * Shooter Subsystem
 *
 * @author Aadit Gupta
 * @author Raadwan Masum
 * @author Rohan Juneja
 */

public class Shooter extends SubsystemBase {
  private CANSparkMax m_shooterMotor;
  private CANEncoder m_encoder;
  private CANPIDController m_controller;

  private PIDController m_pidController;

  private WPI_TalonSRX m_belt_f = new WPI_TalonSRX(Constants.CAN.shooter_belt_front);
  private VictorSPX m_belt_b = new VictorSPX(Constants.CAN.shooter_belt_back);

  private NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry ty = m_table.getEntry("ty");

  /**
   * Constructor for Shooter
   */
  public Shooter() {
    m_shooterMotor = new CANSparkMax(Constants.CAN.shooter_motor, MotorType.kBrushless);
    m_shooterMotor.setIdleMode(IdleMode.kCoast);
    m_shooterMotor.setInverted(true);
    m_encoder = m_shooterMotor.getEncoder();
    m_controller = m_shooterMotor.getPIDController();
    m_controller.setFeedbackDevice(m_encoder);
    m_controller.setP(Constants.Shooter.kP);
    m_controller.setI(Constants.Shooter.kI);
    m_controller.setD(Constants.Shooter.kD);
    stop();

    m_pidController = new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);

    m_belt_b.setInverted(true);
  }

  public double getEncoderValue() {
    return m_encoder.getVelocity();
  }

  /**
   * Set Position of Shooter
   * 
   * @param velocity The desired velocity of the shooter
   */
  public void set(double velocity) {
    m_shooterMotor.set(m_pidController.calculate(m_encoder.getVelocity(), velocity) * 0.001);
    System.out.println(m_pidController.calculate(m_encoder.getVelocity(), velocity) * 0.001);
  }

  /**
   * Stop Shooter
   */
  public void stop() {
    m_controller.setReference(0, ControlType.kDutyCycle);
    m_shooterMotor.stopMotor();
  }

  /**
   * Getting ty value from Limelight and setting velocity to it
   */

  public void visionShoot() {
    double y = ty.getDouble(0.0);

    double vel = y * Constants.Shooter.kVisionP;

    set(vel * Constants.Shooter.kShooterAdjust);
  }

  /**
   * Run belt up
   */
  public void beltUp() {
    m_belt_f.set(ControlMode.PercentOutput, Constants.Shooter.kBeltSpeed);
    m_belt_b.set(ControlMode.PercentOutput, Constants.Shooter.kBeltSpeed);
  }

  /**
   * Run belt down
   */
  public void beltDown() {
    m_belt_f.set(ControlMode.PercentOutput, -Constants.Shooter.kBeltSpeed);
    m_belt_b.set(ControlMode.PercentOutput, -Constants.Shooter.kBeltSpeed);
  }

  /**
   * Stop belt
   */
  public void beltStop() {
    m_belt_f.set(ControlMode.PercentOutput, 0);
    m_belt_b.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Send Powercells to Orbit
   */
  public void sendToOrbit() {
    // m_controller.setReference(Constants.Shooter.maxVoltage, ControlType.kVoltage);
    m_controller.setReference(10, ControlType.kVoltage);
  }
}
