package frc.robot.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Paths {
  public static Path ANSlalomPath = new Path(
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
      new Translation2d(11, 2),
      new Translation2d(5, 2),
      new Translation2d(3, 3.5)
    ),
    // end point
    new Pose2d(1, 4, new Rotation2d(Math.PI))
  );

  public static Path ANBarrelRacingPath = new Path(
    // origin
    new Pose2d(1, 4, new Rotation2d(0)),
    // waypoints
    List.of(
      new Translation2d(6, 4),
      new Translation2d(7.5, 2),
      new Translation2d(6, 2),
      new Translation2d(6.5, 3.5),
      new Translation2d(10, 4),
      new Translation2d(12, 6),
      new Translation2d(10.5, 6.5),
      new Translation2d(10.5, 4),
      new Translation2d(13.5, 2),
      new Translation2d(13.5, 4)
      // new Translation2d(6, 3)
    ),
    // end point
    new Pose2d(1, 4, new Rotation2d(Math.PI))
  );
}
