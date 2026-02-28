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

    public static double getShotAngle(double distanceToTower) {
        double g = 9.8;
        double hubRadius = 0.57;
        double ivanConstant = 0.35;
        double hubHeight = 1.5;
        double velocityS = Math.sqrt(0.5 * g * hubRadius * distanceToTower * (distanceToTower - hubRadius) / Math.abs(hubRadius*hubHeight - ivanConstant*distanceToTower));
        double velocityZ = 0.5*g*distanceToTower/velocityS + hubHeight*velocityS/distanceToTower;
        return Math.PI/2 - Math.atan(velocityZ/velocityS);//
    }
    

    public static double getRawHoodPosition(double distanceToTower) {
        double angleDegrees = getShotAngle(distanceToTower) * 360 / 2 / Math.PI;
        double hoodPosition = (angleDegrees - 22) / 23;
        // for now subtract 1, don't know why
        return hoodPosition - 1;
    }

    public static double getHoodPosition(double distanceToTower) {
        double hoodPosition = getRawHoodPosition(distanceToTower);
        double finalOutput = clamp(0.05, hoodPosition, 0.95);
        return finalOutput;
    }

    public static double getFuelSpeed(double distanceToTower) {
        double g = 9.8;
        double hubRadius = 0.57;
        double ivanConstant = 0.15;
        double hubHeight = 1.5;
        double velocityS = Math.sqrt(0.5 * g * hubRadius * distanceToTower * (distanceToTower - hubRadius) / Math.abs(hubRadius*hubHeight - ivanConstant*distanceToTower));
        double velocityZ = 0.5*g*distanceToTower/velocityS + hubHeight*velocityS/distanceToTower;
        return Math.sqrt(Math.pow(velocityS, 2) + Math.pow(velocityZ, 2));
    }

    public static double getFlywheelSpeed(double distanceToTower) {
        double fuelSpeed = getFuelSpeed(distanceToTower);
        double fuelSpeedToMotorOutput = 0.03131625;
        double flywheelSpeedLossFactor = 1.5; // previously 1.3

        double motorOutput = fuelSpeed * fuelSpeedToMotorOutput * flywheelSpeedLossFactor;
        return -clamp(0, motorOutput, 0.8);
    }
}
