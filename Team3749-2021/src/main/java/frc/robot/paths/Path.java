package frc.robot.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/**
 * Class that stores an autonomous path
 * 
 * @author Rohan
 */
public class Path {
  private final Pose2d origin;

  private final List<Translation2d> waypoints;

  private final Pose2d destination;

  /**
   * Creates a new path object
   * 
   * @param origin      Position/Rotation of origin
   * @param waypoints   List of Points which need to be visited en route to the
   *                    destination
   * @param destination Position/Rotation of destination
   */
  public Path(Pose2d origin, List<Translation2d> waypoints, Pose2d destination) {
    this.origin = origin;
    this.waypoints = waypoints;
    this.destination = destination;
  }

  /**
   * Generates trajectory for the path given the config
   * 
   * @param config An object containing configuration settings for the trajectory
   * @return A Trajectory Object
   */
  public Trajectory getTrajectory(TrajectoryConfig config) {
    return TrajectoryGenerator.generateTrajectory(origin, waypoints, destination, config);
  }
}
