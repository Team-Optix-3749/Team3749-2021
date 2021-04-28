package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Intake Subsystem
 *
 * @author Aadit Gupta
 * @author Raadwan Masum
 * @author Rohan Juneja
 */
public class Intake extends SubsystemBase {
  public VictorSPX m_intake = new VictorSPX(Constants.CAN.intake_motor);
  public VictorSPX m_lift = new VictorSPX(Constants.CAN.intake_motor_lift);

  private final Timer m_timer = new Timer();

  public Intake() {
    m_intake.setInverted(true);
  }

  /**
   * Intake power cells in
   */
  public void intakeIn() {
    m_intake.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
  }

  /**
   * Expel power cells out
   */
  public void intakeOut() {
    m_intake.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);
  }

  /**
   * Stop intake
   */
  public void intakeStop() {
    m_intake.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Lift intake up
   */
  public void liftUp() {
    m_lift.set(ControlMode.PercentOutput, Constants.Intake.kIntakeLiftUpSpeed);
  }

  /**
   * Drop intake down
   */
  public void liftDown() {
    m_lift.set(ControlMode.PercentOutput, Constants.Intake.kIntakeLiftDownSpeed);
  }

  /**
   * Drop intake down with timer
   */
  public void liftDownTimer() {
    m_timer.reset();
    m_timer.start();

    while (m_timer.get() < 0.3)
    m_lift.set(ControlMode.PercentOutput, Constants.Intake.kIntakeLiftDownSpeed);
  }

  /**
   * Stop intake lift
   */
  public void liftStop() {
    m_lift.set(ControlMode.PercentOutput, 0);
  }
}
