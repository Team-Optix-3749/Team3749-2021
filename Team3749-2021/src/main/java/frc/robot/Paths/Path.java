package frc.robot.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Path {
  public Pose2d origin;
  public List<Translation2d> waypoints;
  public Pose2d end;

  public Path(Pose2d origin, List<Translation2d> waypoints, Pose2d end) {
    this.origin = origin;
    this.waypoints = waypoints;
    this.end = end;
  }
}
