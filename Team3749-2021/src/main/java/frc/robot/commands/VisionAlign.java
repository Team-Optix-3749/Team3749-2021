package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An autonomous command to arcade drive
 * 
 * @author Raadwan
 */
public class VisionAlign extends CommandBase {
  private final Drivetrain m_drive;
  private final Shooter m_shooter;
  private final Timer m_timer = new Timer();

  /**
   * 
   * @param drive
   */
  public VisionAlign(Drivetrain drive, Shooter shooter) {
    m_drive = drive;
    m_shooter = shooter;
    addRequirements(m_drive, m_shooter);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

    while (m_timer.get() < 6.0) {
      if (m_timer.get() < 3.0) {
        System.out.println("timer: " + m_timer.get());
        m_drive.visionAlign();
        m_shooter.visionShoot();
      } else if (m_timer.get() > 3.0 && m_timer.get() < 5.0) {
        m_shooter.visionShoot();
        m_shooter.beltUp();
      }
    }
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
