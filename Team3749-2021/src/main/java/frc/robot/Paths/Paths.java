package frc.robot.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Class containing the autonomous paths
 * @author Raadwan
 */

public class Paths {
  public static Path ANSlalomPath = new Path(
    new Pose2d(1, 1, new Rotation2d(0)),
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
    new Pose2d(1, 4, new Rotation2d(Math.PI))
  );

  public static Path ANBarrelRacingPath = new Path(
    new Pose2d(1, 4, new Rotation2d(0)),
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
    ),
    new Pose2d(1, 4, new Rotation2d(Math.PI))
  );

  public static Path ANBouncePath = new Path(
    new Pose2d(1, 4, new Rotation2d(0)),
    List.of(
      new Translation2d(4, 6.5),
      new Translation2d(6, 2),
      new Translation2d(8, 3),
      new Translation2d(8, 6.5),
      new Translation2d(9, 2),
      new Translation2d(12.2, 3),
      new Translation2d(12.2, 6.5)
    ),
    new Pose2d(15, 4, new Rotation2d(0))
  );

  public static Path GSRedPathA = new Path(
    new Pose2d(0.5, 4, new Rotation2d(0)),
    List.of(
      new Translation2d(4, 4),
      new Translation2d(6.5, 3),
      new Translation2d(7.5, 6.5)
    ),
    new Pose2d(15, 7, new Rotation2d(0))
  );

  public static Path GSRedPathB = new Path(
    new Pose2d(0.5, 7.5, new Rotation2d(0)),
    List.of(
      new Translation2d(4, 5.2),
      new Translation2d(6.5, 2.8),
      new Translation2d(10, 6)
    ),
    new Pose2d(15, 7.5, new Rotation2d(0))
  );

  public static Path GSBluePathA = new Path(
    new Pose2d(0.5, 1, new Rotation2d(0)),
    List.of(
      new Translation2d(8, 1.5),
      new Translation2d(9, 5),
      new Translation2d(11, 4.5)
    ),
    new Pose2d(15, 4, new Rotation2d(0))
  );

  public static Path GSBluePathB = new Path(
    new Pose2d(0.5, 1, new Rotation2d(0)),
    List.of(
      new Translation2d(7, 2),
      new Translation2d(11, 5.5),
      new Translation2d(13, 3.5)
    ),
    new Pose2d(15, 2, new Rotation2d(0))
  );
}
