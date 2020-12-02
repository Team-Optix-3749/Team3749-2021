package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();

  public XboxController m_xboxController = new XboxController(0);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(m_xboxController, Axis.kLeftTrigger.value)
        .whenPressed(new Shoot(m_shooter, () -> m_xboxController.getTriggerAxis(Hand.kRight)), true)
        .whenReleased(new Shoot(m_shooter, () -> m_xboxController.getTriggerAxis(Hand.kRight)), true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  /**
   * A simple getter method for the Elevator system
   * 
   * @return m_shooter
   */
  public Elevator getElevator() {
    return m_elevator;
  }

  /**
   * A simple getter method for the Shooter system
   * 
   * @return m_shooter
   */
  public Shooter getShooter() {
    return m_shooter;
  }

  /**
   * A simple getter method for the Intake system
   * 
   * @return m_intake
   */
  public Intake getIntake() {
    return m_intake;
  }
}
