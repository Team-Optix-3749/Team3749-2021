package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
 * @author Rohan Juneja
 */

public class Shooter extends SubsystemBase {
  private CANSparkMax m_shooterMotor;
  private CANEncoder m_encoder;
  private CANPIDController m_controller;

  private VictorSPX m_belt_f = new VictorSPX(Constants.CAN.shooter_belt_front);
  private VictorSPX m_belt_b = new VictorSPX(Constants.CAN.shooter_belt_back);

  // private NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
  // private NetworkTableEntry tx = m_table.getEntry("tx");
  // private NetworkTableEntry ty = m_table.getEntry("ty");
  // private NetworkTableEntry ta = m_table.getEntry("ta");

  /**
   * Constructor for Shooter
   */
  public Shooter() {
    m_shooterMotor = new CANSparkMax(Constants.CAN.shooter_motor, MotorType.kBrushless);
    m_shooterMotor.setIdleMode(IdleMode.kCoast);
    m_encoder = m_shooterMotor.getEncoder();
    m_controller = m_shooterMotor.getPIDController();
    m_controller.setFeedbackDevice(m_encoder);
    m_controller.setP(Constants.Shooter.kP);
    m_controller.setI(Constants.Shooter.kI);
    m_controller.setD(Constants.Shooter.kD);
    stop();
  }

  /**
   * Set Position of Shooter
   * 
   * @param velocity The desired velocity of the shooter
   */
  public void set(double velocity) {
    m_controller.setReference(-velocity, ControlType.kVelocity);
  }

  /**
   * Stop Shooter
   */
  public void stop() {
    m_controller.setReference(0, ControlType.kDutyCycle);
  }

  public void visionAdjust() {
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);
    // double output = 0;

    // output = x * Constants.Drivetrain.kVisionP;

    // output *= Constants.Drivetrain.kVisionLimit;
  }


  /**
   * Run belt up
   */
  public void beltUp() {
    m_belt_f.set(ControlMode.PercentOutput, Constants.Shooter.kBeltSpeed);
    m_belt_b.set(ControlMode.PercentOutput, -Constants.Shooter.kBeltSpeed);
  }

  /**
   * Run belt down
   */
  public void beltDown() {
    m_belt_f.set(ControlMode.PercentOutput, -Constants.Shooter.kBeltSpeed);
    m_belt_b.set(ControlMode.PercentOutput, Constants.Shooter.kBeltSpeed);
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
    m_controller.setReference(Constants.Shooter.maxVoltage, ControlType.kVoltage);
  }
}
