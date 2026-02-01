package frc.robot.util;

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
}
