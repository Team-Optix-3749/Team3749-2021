package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * Shooter belt up
 * 
 * @author Raadwan Masum
 * @author Rohan Juneja
 * @author Aadit Gupta
 */
public class ShooterBeltUp extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Shooter m_shooter;

  public ShooterBeltUp(Shooter subsystem) {
    m_shooter = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_shooter.beltUp();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.beltStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
