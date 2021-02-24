package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

//Constructor for Elevator
public class Elevator extends SubsystemBase {
  private TalonSRX m_elevator;

  //setters for elevator speed; up, down, and stop
  public void upElevator() {
    m_elevator.set(ControlMode.PercentOutput, 1);
  }

  public void downElevator() {
    m_elevator.set(ControlMode.PercentOutput, -1);
  }

  public void stopElevator() {
    m_elevator.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


