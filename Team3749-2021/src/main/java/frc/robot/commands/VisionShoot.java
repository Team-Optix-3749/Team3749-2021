package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An autonomous command to arcade drive
 * 
 * @author Raadwan Masum
 * @author Rohan Juneja
 * @author Aadit Gupta
 */
public class VisionShoot extends CommandBase {
  private final Drivetrain m_drive;
  private final Shooter m_shooter;
  private final Timer m_timer = new Timer();

  /**
   * 
   * @param drive
   */
  public VisionShoot(Drivetrain drive, Shooter shooter) {
    m_drive = drive;
    m_shooter = shooter;
    addRequirements(m_drive, m_shooter);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_drive.visionAlign();

    var shooting = m_shooter.visionShoot();

    System.out.println("shooting: " + shooting);

    if (shooting) {
      m_shooter.beltUp();
    }
  }

  @Override
  public boolean isFinished() {
    return !(m_timer.get() < 6.0 && Robot.m_robotContainer.m_aButton.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stopMotors();
    m_shooter.stop();
    m_shooter.beltStop();
  }
}
