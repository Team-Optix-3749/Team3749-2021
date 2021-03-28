package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Intake extends SubsystemBase {
    private VictorSPX m_intake = new VictorSPX(Constants.CAN.intake_motor);
    private VictorSPX m_lift = new VictorSPX(Constants.CAN.intake_motor_lift);
   
   //Start making the intake suck in powercells
    public void inward(){
      l_intake.set(ControlMode.PercentOutput, 1);
      r_intake.set(ControlMode.PercentOutput, -1);
    }
  //reverse speed to move powercells out
    public void outward(){
      l_intake.set(ControlMode.PercentOutput, -1);
      r_intake.set(ControlMode.PercentOutput, 1);
    }
  //sets speed back to rest at 0 
    public void stop(){
      l_intake.set(ControlMode.PercentOutput, 0);
      r_intake.set(ControlMode.PercentOutput, 0);
    }

    public void antiJam(){

    }
}
