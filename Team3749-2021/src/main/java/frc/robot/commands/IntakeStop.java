package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Stop intake
 * 
 * @author Raadwan Masum
 * @author Aadit Gupta
 * @author Rohan Juneja
 */
public class IntakeStop extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Intake m_intake;

  public IntakeStop(Intake subsystem) {
    m_intake = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_intake.intakeStop();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.intakeStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
