package frc.robot;

import frc.robot.paths.*;

public final class Constants {
  public static class CAN {
    public static final int drive_lf = 11;
    public static final int drive_lb = 20;
    public static final int drive_rf = 13;
    public static final int drive_rb = 23;
    public static final int shooter_motor = 10;
    public static final int intake_motor = 21;
    public static final int intake_motor_lift = 22;
    public static final int elevator_motor = 24;
    public static final int control_panel_motor = 15;

  }

  // Whether a subsystem is installed and in use
  // 0 = disabled, 1 = enabled, 2 = enabled and debugging (print sensor vals, etc)
  public static class Safety {
    public static final int drive = 1;
    public static final int shooter = 1;
    public static final int intake = 1;
    public static final int elevator = 1;
    public static final int controlPanel = 1;
    public static final int vision = 1;
  }

  public static class Drivetrain {
    public static final double kDriveSpeed = 1.0;
    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = Math.PI;
    public static final double kTrackWidth = 0.381 * 2;
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = -4096;
  }

  public static class Shooter {
    public static final double kP = 1.0;
    public static final double kI = 1.0;
    public static final double kD = 1.0;
    public static final int maxRPM = 5500;
    public static final int maxVoltage = 12;
  }

  public static class Autonomous {
    public static final double kB = 2.0;
    public static final double kZeta = 0.7;
    public static final Path AutoPath = Paths.ANSlalomPath;
  }
}
