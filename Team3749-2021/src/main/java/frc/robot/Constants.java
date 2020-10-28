package frc.robot;

import java.util.HashMap;

public final class Constants {
    public static class CAN {
        static int drive_lf = 11;
        static int drive_lb = 20;
        static int drive_rf = 13;
        static int drive_rb = 23;
        static int shooter_motor = 10;
        static int ntake_motor_f = 21;
        static int intake_motor_b = 22;
        static int elevator_motor = 24;
        static int control_panel_motor = 15;

    }

    // Whether a subsystem is installed and in use
    // 0 = disabled, 1 = enabled, 2 = enabled and debugging (print sensor vals, etc)
    public static class Safety {
        static int drive = 1;
        static int shooter = 1;
        static int intake = 1;
        static int elevator = 1;
        static int controlPanel = 1;
        static int vision = 1;
    }
}
