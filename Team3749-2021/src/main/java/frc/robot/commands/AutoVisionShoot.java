package frc.robot.commands;

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
public class AutoVisionShoot extends CommandBase {
  private final Drivetrain m_drive;
  private final Shooter m_shooter;
  private final Timer m_timer = new Timer();

  /**
   * 
   * @param drive
   */
  public AutoVisionShoot(Drivetrain drive, Shooter shooter) {
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
    return m_timer.get() > 3.0;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stopMotors();
    m_shooter.stop();
    m_shooter.beltStop();
  }
}
