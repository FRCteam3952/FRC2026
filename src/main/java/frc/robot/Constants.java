package frc.robot;
// luca lopez class of 2026 did all of this and is software king
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public class Ports {
        // spark flexes
        public static final int FLYWHEEL2_CAN_ID = 4; // switch if it goes backwards
        public static final int FLYWHEEL_CAN_ID = 3;
        public static final int AGITATOR_CAN_ID = 1; 
        
        // normal sparks
        public static final int LOWER_INTAKE_CAN_ID = 7;
        
        public static final int RIGHT_INTAKE_PIVOT_CAN_ID = 14;

        public static final int UPPER_INTAKE_CAN_ID = 17; 
        public static final int ROLLER_CAN_ID = 21;
        public static final int INTAKE_MOTOR_CAN_ID = 19;


        // talons
        public static final int HOOD_COVER_CAN_ID = 24;
        // public static final int LEFT_INTAKE_PIVOT_CAN_ID = 24;
        public static final int CLIMBER_CAN_ID = 25; // doesnt exist
    }

    public static class NetworkTablesConstants {
		public static final String MAIN_TABLE_NAME = "robot";
	}

    public static class FieldConstants {
        // TODO: this probably isn't relatively correct
        // next try 0 for y, because that might be the middle
        public static final Translation2d RED_HUB_CENTER_TRANSLATION = new Translation2d(11.9150, 4.0345);
        public static final Translation2d BLUE_HUB_CENTER_TRANSLATION = new Translation2d(4.6256, 4.0345);
    }

    public static class RobotConstants {
        public static final double minShooterAngle = 22.5;
        public static final double maxShooterAngle = 45.0;
    }
}