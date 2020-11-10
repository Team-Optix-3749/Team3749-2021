package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;



public class Intake extends SubsystemBase {
    private VictorSPX l_intake;
    private VictorSPX r_intake;

    public Intake(){

    }

    public void inward(){
      l_intake.set(ControlMode.PercentOutput, 1);
      r_intake.set(ControlMode.PercentOutput, -1);
    }

    public void outward(){
      l_intake.set(ControlMode.PercentOutput, -1);
      r_intake.set(ControlMode.PercentOutput, 1);
    }

    public void antiJam(){

    }
}
