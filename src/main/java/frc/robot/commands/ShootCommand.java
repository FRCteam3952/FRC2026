package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.KinematicsUtil;

public class ShootCommand extends Command {
    // doenst aim yaw
    ShooterSubsystem shooter;
    Timer t = new Timer();

    boolean shooterLoading = false;

    public ShootCommand(ShooterSubsystem s) {
        this.shooter = s;

        addRequirements(s);
    }

    @Override
    public void initialize() {
        t.restart();
    }

    @Override
    public void execute() {
        Pose2d botPose = RobotContainer.INSTANCE.getBotPose();
        ChassisSpeeds currentSpeed = RobotContainer.INSTANCE.getSpeeds();
        
        var shooterState = KinematicsUtil.getShooterState(botPose.getX(), botPose.getY(), currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);
        LinearVelocity flywheelVelocty = shooterState.getFirst();
        Angle hoodAngle = shooterState.getSecond().getSecond();
        
        // Auto aim shooter
        shooter.setHoodSetpoint(hoodAngle);
        shooter.setFlywheelSpeed(flywheelVelocty);

        if (t.hasElapsed(1.0) && !shooterLoading) {
            shooterLoading = true;
            shooter.startLoadFuel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (shooterLoading) {
            shooter.stopLoadFuel();
        }
    }

    @Override
    public boolean isFinished() {
        return false;   
    }
}
