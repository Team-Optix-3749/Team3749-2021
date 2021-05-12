package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class Auto2020 extends SequentialCommandGroup {
  public Auto2020(Drivetrain m_drive, Shooter m_shooter) {
    addCommands(
      new AutoVisionShoot(m_drive, m_shooter),
      new AutoPath(m_drive, List.of(Constants.Paths.Auto2020_1)), 
      new WaitCommand(5.0),
      new AutoPath(m_drive, List.of(Constants.Paths.Auto2020_2))
     );
  }
}
