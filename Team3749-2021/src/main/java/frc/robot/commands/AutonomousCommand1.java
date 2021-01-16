package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

import java.util.List;

public class AutonomousCommand1 extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  private Trajectory m_trajectory = new Trajectory();
  private final RamseteController m_follower;
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  public AutonomousCommand1(Drivetrain drivetrain) {
    m_drive = drivetrain;

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(10.0), Units.feetToMeters(10.0));
    config.setKinematics(m_drive.getKinematics());

    m_trajectory = TrajectoryGenerator.generateTrajectory(
    // origin
    new Pose2d(1, 1, new Rotation2d(0)),
    // waypoints
    List.of(
      new Translation2d(5, 3.5),
      new Translation2d(11, 3.5),
      new Translation2d(13, 2),
      new Translation2d(14.5, 2.5),
      new Translation2d(14.5, 3.5),
      new Translation2d(13, 3.5),
      new Translation2d(11, 1.5),
      new Translation2d(5, 1.5),
      new Translation2d(3, 3.5)
    ),
    // end point
    new Pose2d(1, 3.5, new Rotation2d(0)),
    // config
    config);

    m_drive.resetOdometry(m_trajectory.getInitialPose());

    m_follower = new RamseteController(Constants.Autonomous.kB, Constants.Autonomous.kZeta);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_prevTime = -1;
    var initialState = m_trajectory.sample(0);
    var angularVelocity = initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond;
    m_prevSpeeds = m_drive.getKinematics()
        .toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0, angularVelocity));
    m_timer.reset();
    m_timer.start();
    m_drive.getLeftPIDController().reset();
    m_drive.getRightPIDController().reset();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    if (m_prevTime < 0) {
      m_drive.setOutputVolts(0.0, 0.0);
      m_prevTime = curTime;
      return;
    }

    var targetWheelSpeeds = m_drive.getKinematics()
        .toWheelSpeeds(m_follower.calculate(m_drive.getPose(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    var speeds = m_drive.getSpeeds();

    double leftFeedforward = m_drive.getFeedforward().calculate(leftSpeedSetpoint,
        (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

    double rightFeedforward = m_drive.getFeedforward().calculate(rightSpeedSetpoint,
        (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

    double leftOutput = leftFeedforward
        + m_drive.getLeftPIDController().calculate(speeds.leftMetersPerSecond, leftSpeedSetpoint);

    double rightOutput = rightFeedforward
        + m_drive.getRightPIDController().calculate(speeds.rightMetersPerSecond, rightSpeedSetpoint);

    m_drive.setOutputVolts(leftOutput, rightOutput);
    m_prevSpeeds = targetWheelSpeeds;
    m_prevTime = curTime;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setOutputVolts(0, 0);
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
