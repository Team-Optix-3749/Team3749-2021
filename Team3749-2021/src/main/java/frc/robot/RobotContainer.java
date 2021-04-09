package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final Drivetrain m_drive = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  // private final AutoBarrelRacingPath m_autoCommand = new AutoBarrelRacingPath(m_drive);
  // private final AutoBouncePath m_autoCommand = new AutoBouncePath(m_drive);
  // private final AutoSearchPathA m_autoCommand = new AutoSearchPathA(m_drive, m_intake);
  // private final AutoSearchPathB m_autoCommand = new AutoSearchPathB(m_drive, m_intake);
  // private final AutoBouncePath1 m_autoBouncePath1 = new AutoBouncePath1(m_drive);
  // private final AutoBouncePath2 m_autoBouncePath2 = new AutoBouncePath2(m_drive);
  // private final AutoBouncePath3 m_autoBouncePath3 = new AutoBouncePath3(m_drive);
  // private final AutoBouncePath4 m_autoBouncePath4 = new AutoBouncePath4(m_drive);
  // private final SequentialCommandGroup m_autoCommand = new SequentialCommandGroup(m_autoBouncePath1, m_autoBouncePath2, m_autoBouncePath3, m_autoBouncePath4);
  private final AutoSlalomPath1 m_autoSlalomPath1 = new AutoSlalomPath1(m_drive);
  private final AutoSlalomPath2 m_autoSlalomPath2 = new AutoSlalomPath2(m_drive);
  private final SequentialCommandGroup m_autoCommand = new SequentialCommandGroup(m_autoSlalomPath1, m_autoSlalomPath2);

  public XboxController m_xboxController = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();

    // drivetrain controller bindings
    m_drive.setDefaultCommand(
        new ArcadeDrive(m_drive, () -> m_xboxController.getY(Hand.kLeft), () -> m_xboxController.getX(Hand.kRight)));
  }

  private void configureButtonBindings() {
    // shooter controller bindings
    // new JoystickButton(m_xboxController, Axis.kRightTrigger.value)
    // .whenPressed(new Shoot(m_shooter, () ->
    // m_xboxController.getTriggerAxis(Hand.kRight)), true)
    // .whenReleased(new ShootStop(m_shooter), true);
    // .whenReleased(new Shoot(m_shooter, () ->
    // m_xboxController.getTriggerAxis(Hand.kRight)), true);

    // shooter controller bindings
    // new JoystickButton(m_xboxController, Button.kA.value).toggleWhenPressed(new
    // ShootMax(m_shooter), true);
    // new JoystickButton(m_xboxController, Button.kA.value).whenPressed(new
    // ShootMax(m_shooter), true)
    // .whenReleased(new ShootStop(m_shooter), true);

    new JoystickButton(m_xboxController, Button.kA.value).toggleWhenPressed(new ShootMax(m_shooter), true);

    new JoystickButton(m_xboxController, Button.kBumperLeft.value).whenPressed(new ShooterBeltUp(m_shooter), true)
        .whenReleased(new ShooterBeltStop(m_shooter), true);

    // intake in controller bindings
    new JoystickButton(m_xboxController, Button.kBumperRight.value).toggleWhenPressed(new IntakeIn(m_intake), true);

    // intake up controller bindings
    new JoystickButton(m_xboxController, Button.kB.value).whenPressed(new IntakeUp(m_intake), true)
        .whenReleased(new IntakeLiftStop(m_intake), true);

    // intake down controller bindings
    new JoystickButton(m_xboxController, Button.kX.value).whenPressed(new IntakeDown(m_intake), true)
        .whenReleased(new IntakeLiftStop(m_intake), true);

    // belt up controller bindings
    // new JoystickButton(m_xboxController,
    // Button.kBumperLeft.value).toggleWhenPressed(new ShooterBeltUp(m_shooter),
    // true);

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
