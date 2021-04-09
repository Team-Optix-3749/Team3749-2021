package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.paths.*;

import java.io.IOException;
// import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * An autonomous command to follow a path
 * 
 * @author Rohan
 * @author Raadwan
 */
public class AutoBouncePath2 extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  private Trajectory m_trajectory;
  private final RamseteController m_follower;
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  /**
   * Creates an ``AutoNavPath`` object
   * 
   * @param drivetrain The drivetrain subsystem
   * @param path       The path to follow
   */
  public AutoBouncePath2(Drivetrain drivetrain) {
    m_drive = drivetrain;

    String trajectoryJSON = "paths/output/bounce.wpilib.json";

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(10.0), Units.feetToMeters(10.0));
    config.setKinematics(m_drive.getKinematics());

    // m_trajectory = path.getTrajectory(config);

    try {
      Path m_trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(m_trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      System.out.println("Unable to open trajectory");
    }

    m_drive.resetOdometry(m_trajectory.getInitialPose());

    m_follower = new RamseteController(Constants.Autonomous.kB, Constants.Autonomous.kZeta);

    addRequirements(drivetrain);
  }

  /**
   * Runs when the command is initialized
   */
  @Override
  public void initialize() {
    m_drive.resetOdometry(m_trajectory.getInitialPose());
    m_prevTime = -1;
    var initialState = m_trajectory.sample(0);
    var angularVelocity = initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond;
    m_prevSpeeds = m_drive.getKinematics()
        .toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0, angularVelocity));
    m_timer.reset();
    m_timer.start();
    m_drive.getLeftPIDController().reset();
    m_drive.getRightPIDController().reset();
    m_drive.m_leftEncoder.reset();
    m_drive.m_rightEncoder.reset();
  }

  /**
   * Runs when the command is executed
   */
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

    m_drive.setOutputVolts(leftOutput, -rightOutput);
    m_prevSpeeds = targetWheelSpeeds;
    m_prevTime = curTime;
  }

  /**
   * Runs when the command is stopped
   */
  @Override
  public void end(boolean interrupted) {
    m_drive.setOutputVolts(0, 0);
    m_timer.stop();
  }

  /**
   * Returns whether the command has been completed
   */
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
