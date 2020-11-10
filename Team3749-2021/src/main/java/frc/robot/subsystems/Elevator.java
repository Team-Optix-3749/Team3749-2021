package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Elevator extends SubsystemBase {
  private TalonSRX m_elevator;

  public Elevator() {

  }

  public void upElevator() {
    m_elevator.set(ControlMode.PercentOutput, 1);
  }

  public void downElevator() {
    m_elevator.set(ControlMode.PercentOutput, -1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


