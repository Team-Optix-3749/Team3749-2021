package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Stop intake lift
 * 
 * @author Raadwan Masum
 * @author Aadit Gupta
 * @author Rohan Juneja
 */
public class IntakeLiftStop extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Intake m_intake;

  public IntakeLiftStop(Intake subsystem) {
    m_intake = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_intake.liftStop();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.liftStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
