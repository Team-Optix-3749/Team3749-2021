package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 */
public class AutoBarrelRacingPath extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drive;

  /**
   *
   * @param Drivetrain The subsystem used by this command.
   */
  public AutoBarrelRacingPath(Drivetrain subsystem) {
    m_drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.auto(1.0, 1.0, 1.0);
    m_drive.auto(0.5, 1.0, 0.5);
    m_drive.auto(1.0, 0.5, 0.5);
    m_drive.auto(1.0, 1.0, 1.0);
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
