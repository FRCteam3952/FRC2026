package frc.robot.util;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ControlUtils {
    public static double clamp(double input) {
        if (input > 1.0) {
            return 1.0;
        }
        if (input < -1.0) {
            return -1.0;
        }
        return input;
    }

    public static double clamp(double min, double input, double max) {
        if (input > max) {
            return max;
        }
        if (input < min) {
            return min;
        }
        return input;
    }
}
