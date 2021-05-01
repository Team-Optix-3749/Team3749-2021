package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An autonomous command to arcade drive
 * 
 * @author Raadwan Masum
 * @author Rohan Juneja
 * @author Aadit Gupta
 */
public class VisionStop extends CommandBase {
  private final Drivetrain m_drive;
  private final Shooter m_shooter;

  /**
   * 
   * @param drive
   */
  public VisionStop(Drivetrain drive, Shooter shooter) {
    m_drive = drive;
    m_shooter = shooter;
    addRequirements(m_drive, m_shooter);
  }

  @Override
  public void initialize() {
    m_drive.stopMotors();
    m_shooter.stop();
    m_shooter.beltStop();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stopMotors();
    m_shooter.stop();
    m_shooter.beltStop();
  }
}
