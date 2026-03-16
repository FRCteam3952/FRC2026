package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.lang.reflect.Field;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.FieldConstants;

public class KinematicsUtil {
    public static Pair<LinearVelocity, Pair<Angle, Angle>> getShooterState(double xPos, double yPos, double xSpeed, double ySpeed) {
        // System.out.println("xPos: " + xPos + ", yPos: " + yPos + ", xSpeed: " + xSpeed + ", ySpeed: " + ySpeed);
        double g = 9.8;
        double hubRadius = 0.57;
        double ivanConstant = 0.15;
        double hubHeight = 1.5; // negating shooter hHeight
        double hubX = FieldConstants.BLUE_HUB_CENTER_TRANSLATION.getX();
        double hubY = FieldConstants.BLUE_HUB_CENTER_TRANSLATION.getY();
        
        double xDiff = xPos-hubX;
        double yDiff = yPos-hubY;
        
        double distanceToHub = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
        double y1 = hubHeight + ivanConstant;
        double y2 = hubHeight;
        double x1 = distanceToHub - hubRadius;
        double x2 = distanceToHub;
        
        double hubYaw = Math.atan(yDiff / xDiff);
        
        // static shot angle, speed, xy speeds
        double sSA = Math.atan((y2*x1*x1-y1*x2*x2)/(x1*x2*(x1-x2)));
        double sSS = x1*Math.sqrt(g/(2*Math.cos(sSA)*(x1*Math.sin(sSA)-y1*Math.cos(sSA))));
        double sSxS = sSS * Math.cos(sSA) * Math.cos(hubYaw);
        double sSyS = sSS * Math.cos(sSA) * Math.sin(hubYaw);
        
        // moving shot xyz speeds, yaw, speed, hood angle
        double mSxS = sSxS - xSpeed;
        double mSyS = sSyS - ySpeed;
        double mSzS = sSS * Math.sin(sSA);
        double mYaw = Math.atan(mSyS/mSxS);
        double mSS = Math.sqrt(Math.pow(mSxS, 2) + Math.pow(mSyS, 2) + Math.pow(mSzS, 2));
        double mSA = Math.atan(mSzS/Math.sqrt(Math.pow(mSxS, 2) + Math.pow(mSyS, 2)));

        Angle yawAngle = Angle.ofBaseUnits(mYaw, Radians);
        Angle hoodAngle = Angle.ofBaseUnits(mSA, Radians);
        LinearVelocity flywheelVelocity = LinearVelocity.ofBaseUnits(mSS, MetersPerSecond);

        // System.out.println("yawAngle: " + yawAngle + ", hoodAngle: " + hoodAngle + ", flywheelVelocity: " + flywheelVelocity);
        return new Pair<LinearVelocity, Pair<Angle,Angle>>(flywheelVelocity, new Pair<Angle, Angle>(yawAngle, hoodAngle));
    }
}
