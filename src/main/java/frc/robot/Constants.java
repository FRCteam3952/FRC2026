package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class Ports {
        // public static int TURRET_FLYWHEEL_PORT = 22;
        // public static int TURRET_ROTATE_PORT = 5;
        // public static int TURRET_LOWER_INTAKE_PORT = 17;
        // public static int TURRET_UPPER_INTAKE_PORT = 13;
        public static int UPPER_INTAKE_PORT = 5; 
        public static int LOWER_INTAKE_CAN_ID = 17;
        public static int ROLLER_CAN_ID = 3;
        public static int AGITATOR_CAN_ID = 4;
        public static int FLYWHEEL_CAN_ID = 20;
        public static int UPPER_INTAKE_CAN_ID = 21;
        public static int HOOD_COVER_CAN_ID = 22;

        public static int INTAKE_MOTOR_CAN_ID = 19;
        // public static int PIVOT_MOTOR_ID = ; // idk
    }

    public static class NetworkTablesConstants {
		public static final String MAIN_TABLE_NAME = "robot";
	}

    public static class FieldConstants {
        // TODO: this probably isn't relatively correct
        // next try 0 for y, because that might be the middle
        public static final Translation2d HUB_CENTER_TRANSLATION = new Translation2d(11.00, 4.25);
    }
}