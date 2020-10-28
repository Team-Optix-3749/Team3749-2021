package frc.robot;

import java.util.HashMap;

public final class Constants {
    public static class CAN {
        public static int drive_lf = 11;
        public static int drive_lb = 20;
        public static int drive_rf = 13;
        public static int drive_rb = 23;
        public static int shooter_motor = 10;
        public static int ntake_motor_f = 21;
        public static int intake_motor_b = 22;
        public static int elevator_motor = 24;
        public static int control_panel_motor = 15;

    }

    // Whether a subsystem is installed and in use
    // 0 = disabled, 1 = enabled, 2 = enabled and debugging (print sensor vals, etc)
    public static class Safety {
        public static int drive = 1;
        public static int shooter = 1;
        public static int intake = 1;
        public static int elevator = 1;
        public static int controlPanel = 1;
        public static int vision = 1;
    }
}
