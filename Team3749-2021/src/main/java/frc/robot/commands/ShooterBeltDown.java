package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * Belt down shooter
 * 
 * @author Raadwan Masum
 * @author Rohan Juneja
 * @author Aadit Gupta
 */
public class ShooterBeltDown extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Shooter m_shooter;

  public ShooterBeltDown(Shooter subsystem) {
    m_shooter = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_shooter.beltDown();
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
