package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Ports;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NetworkTablesUtil;

public class LimelightSubsystem extends SubsystemBase {
    private final double delayBetweenLimelightReadings = 0.020; // 20 ms
    private final String[] limelightNames = {"limelight-a", "limelight-b", "limelight-c"};
    
    // Setting this to 0.02 as a default value for the first tick so stuff doesn't break; shouldn't matter.
    private double timeSinceLastTick = 0.02;
    private double timeAtLastTick = 0.0;
    private double timeSinceLastReading = 0.0;

    private CommandSwerveDrivetrain drivetrain;

    public LimelightSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // TODO: Have an odometry thread, instead of doing all the odometry stuff here.
        double currentTime = Timer.getFPGATimestamp();
        this.timeSinceLastTick = Math.max(0, currentTime - this.timeAtLastTick);
        this.timeAtLastTick = currentTime;

        this.timeSinceLastReading += this.timeSinceLastTick;

        if (this.timeSinceLastReading >= this.delayBetweenLimelightReadings) {
            this.timeSinceLastReading = 0.0;
            updateOdometryWithLimelightOutput();
        }
        updateMegaTag2RobotYaw();
    }

    public void updateOdometryWithLimelightOutput() {
        for (String limelightName : limelightNames) {
            // Get the pose estimate
            // TODO: shouldn't be hardcoded blue i think
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            // Didn't find a tag
            if (mt2.tagCount == 0) {
                continue;
            }
            // Spinning too fast
            if (drivetrain.getState().Speeds.omegaRadiansPerSecond > 2 * Math.PI) {
                continue;
            }
            
            // System.out.println(mt2.pose);

            // Add it to your pose estimator
            // TODO: do we tune these stdevs?
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds
            );
        }
    }

    public void updateMegaTag2RobotYaw() {
        double robotYaw = this.drivetrain.getPigeon2().getYaw().getValue().in(Degrees);
        LimelightHelpers.SetRobotOrientation("limelight-a", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetRobotOrientation("limelight-b", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetRobotOrientation("limelight-c", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}
