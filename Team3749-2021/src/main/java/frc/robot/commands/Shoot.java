package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * An example command that uses an example subsystem.
 */
public class Shoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private final DoubleSupplier m_controllerInput;

  /**
   * Creates a new Shoot Command.
   *
   * @param subsystem The Shooter used by this command
   * @param controllerInput A double supplier providing the encoder input ( Should be 0-1, will be scaled to 0-5500 RPM )
   */
  public Shoot(Shooter shooter, DoubleSupplier controllerInput) {
    m_shooter = shooter;
    m_controllerInput = controllerInput;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.set(m_controllerInput.getAsDouble() * 5500);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
