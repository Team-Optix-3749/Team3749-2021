package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 */
public class AutoSlalomPath extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drive;

  /**
   *
   * @param Drivetrain The subsystem used by this command.
   */
  public AutoSlalomPath(Drivetrain subsystem) {
    m_drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // first turn
    m_drive.auto(1.0, 1.0, 0.2); // forward
    m_drive.auto(0.5, 1.0, 0.7); // left
    m_drive.auto(1.0, 1.0, 0.2); // forward
    m_drive.auto(1.0, 0.5, 0.6); // right

    // first straight
    m_drive.auto(1.0, 1.0, 0.9); // forward

    // roundabout
    m_drive.auto(1.0, 0.5, 0.6); // right
    m_drive.auto(0.7, 1.0, 0.4); // left
    m_drive.auto(0.5, 0.5, 0.4); // forward
    m_drive.auto(0.7, 1.0, 0.4); // left
    m_drive.auto(0.5, 0.5, 0.4); // forward
    m_drive.auto(0.7, 1.0, 0.4); // left
    m_drive.auto(0.5, 0.5, 0.4); // forward
    m_drive.auto(0.7, 1.0, 0.4); // left
    m_drive.auto(0.5, 0.5, 0.4); // forward

    // second straight
    m_drive.auto(1.0, 1.0, 0.9); // forward

    // second turn
    m_drive.auto(0.5, 1.0, 0.7); // right
    m_drive.auto(1.0, 1.0, 0.2); // forward
    m_drive.auto(0.5, 1.0, 0.7); // left
    m_drive.auto(1.0, 1.0, 0.3); // forward
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.stopMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
