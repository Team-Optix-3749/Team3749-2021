package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

  /**
   * Constructor
   */
  public Shooter() {
    m_shooterMotor = new CANSparkMax(Constants.CAN.shooter_motor, MotorType.kBrushless);
    m_shooterMotor.setIdleMode(IdleMode.kCoast);
    m_encoder = m_shooterMotor.getEncoder();
    m_controller = m_shooterMotor.getPIDController();
    m_controller.setFeedbackDevice(m_encoder);
    m_controller.setP(Constants.ShooterGains.kP);
    m_controller.setI(Constants.ShooterGains.kI);
    m_controller.setD(Constants.ShooterGains.kD);
    stop();
  }

  /**
   * Set Position of Shooter
   */

  public void set(double velocity) {
    m_controller.setReference(velocity, ControlType.kVelocity);
  }

  /**
   * Stop Shooter
   */

  public void stop() {
    m_controller.setReference(0, ControlType.kDutyCycle);
  }

  /**
   * Send Powercells to Orbit
   */

  public void sendToOrbit() {

  }

  /**
   * Un-Jam Powercells
   */

  public void antiJam() {

  }
}
