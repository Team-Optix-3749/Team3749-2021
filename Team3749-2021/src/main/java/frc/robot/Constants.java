package frc.robot;

import java.util.HashMap;

public final class Constants {
    public static class CAN {
        public static final int drive_lf = 11;
        public static final int drive_lb = 20;
        public static final int drive_rf = 13;
        public static final int drive_rb = 23;
        public static final int shooter_motor = 10;
        public static final int ntake_motor_f = 21;
        public static final int intake_motor_b = 22;
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
}
