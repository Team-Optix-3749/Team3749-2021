package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final Drivetrain m_drive = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final AutoNavPath m_autoCommand = new AutoNavPath(m_drive, Constants.Autonomous.AutoPath);

  public XboxController m_xboxController = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();

    // drivetrain controller bindings
    m_drive.setDefaultCommand(
        new ArcadeDrive(m_drive, () -> m_xboxController.getY(Hand.kLeft), () -> m_xboxController.getX(Hand.kRight)));
  }

  private void configureButtonBindings() {
    // shooter controller bindings
    new JoystickButton(m_xboxController, Button.kA.value)
        .whenPressed(new Shoot(m_shooter, () -> m_xboxController.getTriggerAxis(Hand.kRight)), true)
        .whenReleased(new Shoot(m_shooter, () -> m_xboxController.getTriggerAxis(Hand.kRight)), true);

    // intake in controller bindings
    new JoystickButton(m_xboxController, Button.kB.value).toggleWhenPressed(new IntakeIn(m_intake), true);

    // intake up controller bindings
    new JoystickButton(m_xboxController, Button.kStart.value).whenPressed(new IntakeUp(m_intake), true)
        .whenReleased(new IntakeLiftStop(m_intake), true);

    // intake down controller bindings
    new JoystickButton(m_xboxController, Button.kBack.value).whenPressed(new IntakeDown(m_intake), true)
        .whenReleased(new IntakeLiftStop(m_intake), true);

    // belt up controller bindings
    new JoystickButton(m_xboxController, Button.kX.value).whenPressed(new ShooterBeltUp(m_shooter), true)
        .whenReleased(new ShooterBeltStop(m_shooter), true);

    // belt down controller bindings
    new JoystickButton(m_xboxController, Button.kY.value).whenPressed(new ShooterBeltDown(m_shooter), true)
        .whenReleased(new ShooterBeltStop(m_shooter), true);
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
   * A simple getter method for the Drivetrain system
   * 
   * @return m_drive
   */
  public Drivetrain getDrivetrain() {
    return m_drive;
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

  public void reset() {
    m_drive.reset();
  }
}
