package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * An ShootMax command that uses Shooter subsystem.
 * 
 * @author Raadwan Masum
 * @author Rohan Juneja
 * @author Aadit Gupta
 */
public class ShootMax extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Shooter m_shooter;

  public ShootMax(Shooter subsystem) {
    m_shooter = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_shooter.sendToOrbit();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
