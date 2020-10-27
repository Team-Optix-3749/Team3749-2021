package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
