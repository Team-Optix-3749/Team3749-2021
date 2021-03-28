package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Intake extends SubsystemBase {
  private VictorSPX m_intake = new VictorSPX(Constants.CAN.intake_motor);
  private VictorSPX m_lift = new VictorSPX(Constants.CAN.intake_motor_lift);

  /**
   * Intake power cells in
   */
  public void intakeIn() {
    m_intake.set(ControlMode.PercentOutput, 1);
  }

  /**
   * Expel power cells out
   */
  public void intakeOut() {
    m_intake.set(ControlMode.PercentOutput, -1);
  }

  /**
   * Lift intake up
   */
  public void liftUp() {
    m_lift.set(ControlMode.PercentOutput, 1);
  }

  /**
   * Drop intake down
   */
  public void liftDown() {
    m_lift.set(ControlMode.PercentOutput, -1);
  }
}
