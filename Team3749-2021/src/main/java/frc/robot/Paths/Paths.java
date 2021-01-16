package frc.robot.Paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Paths {

  public static Path ANSlalom = new Path(
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
    new Pose2d(1, 3.5, new Rotation2d(0))
  );
}
