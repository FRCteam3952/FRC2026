// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import edu.wpi.first.wpilibj2.command.button.;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.ControlUtils;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NetworkTablesUtil;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private Command teleopDriveCommmand;
    private Command followApriltagCommand;
    

    // private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandPS5Controller joystick = new CommandPS5Controller(0);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelights = new LimelightSubsystem(drivetrain);
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    
    // public final TurretSubsystem deniTurret = new TurretSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Initialize limelights and drivetrain yaw for odometry
        // TODO: what do the different modes do, exactly?
        LimelightHelpers.SetIMUMode("limelight-a", 1);
        LimelightHelpers.SetIMUMode("limelight-b", 1);
        LimelightHelpers.SetIMUMode("limelight-c", 1);
        limelights.updateMegaTag2RobotYaw();

        teleopDriveCommmand = drivetrain.applyRequest(() -> {
            // System.out.println("Apriltag Entry A: " + NetworkTablesUtil.getAprilTagEntry('a'));
            // System.out.println("Apriltag Entry B: " + NetworkTablesUtil.getAprilTagEntry('b'));
            // System.out.println("Apriltag Entry C: " + NetworkTablesUtil.getAprilTagEntry('c'));

            return drive.withVelocityX(Math.pow(joystick.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward) (MaxSpeed)
                .withVelocityY(Math.pow(joystick.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left) (MaxSpeed)
                .withRotationalRate(-joystick.getRightX()*MaxAngularRate)//-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                .withDeadband(0.1);
            });
    
        followApriltagCommand = drivetrain.applyRequest(() -> {
            {
            // Optional<double[]> optionalBotPose = NetworkTablesUtil.getAprilTagEntry('b');

            // if (optionalBotPose.isEmpty()) {
            //     System.out.println("no bot pose :(");

            //     return drive.withVelocityX(0) // Drive forward with negative Y (forward) (MaxSpeed)
            //         .withVelocityY(0) // Drive left with negative X (left) (MaxSpeed)
            //         .withRotationalRate(0);//-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            // }

            // double[] botPose = optionalBotPose.get();

            // double currentX = botPose[0]; // probably correct
            // double currentY = botPose[1];
            // double currentYaw = botPose[5];
            // System.out.println("currentX: " + currentX + " & expectedX = " + drivetrain.getExpectedX()+"\n"
            // +"currentY: " + currentY + " & expectedY = " + drivetrain.getExpectedY()+"\n"
            // +"currentYaw: " + currentYaw + " & expectedYaw = " + 0);
            }

            Pose2d botPose = drivetrain.getState().Pose;

            {
            // For some reason we keep getting (0, 0) when the limelight doesn't detect the apriltag, even though that's not supposed to be the default.
            // This code is here to fix that.
            // if (currentX == 0 && currentY == 0) {
            //     return drive.withVelocityX(0) // Drive forward with negative Y (forward) (MaxSpeed)
            //         .withVelocityY(0) // Drive left with negative X (left) (MaxSpeed)
            //         .withRotationalRate(0);//-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            // 
            }

            // double expectedX = drivetrain.getExpectedX();
            // double expectedY = drivetrain.getExpectedY();

            double maxSpeed = 0.1;

            double kPYaw = 0.4;
            double maxRotationalSpeed = 0.2;

            double towerX = 12.00; // this is made up and not a real number at all.
            double towerY = 4.00;

            // Auto-target the tower code
            double xDifference = towerX - botPose.getX();
            double yDifference = towerY - botPose.getY();
            double expectedYaw = -Math.atan(xDifference / yDifference); // clockwise is positive
            if (xDifference < 0) {
                expectedYaw = 90 - expectedYaw;
            }


            double xSpeed = -ControlUtils.clamp(drivetrain.getXSpeed(botPose.getX())) * maxSpeed;
            double ySpeed = -ControlUtils.clamp(drivetrain.getYSpeed(botPose.getY())) * maxSpeed;
            double yawSpeed = ControlUtils.clamp((expectedYaw - botPose.getRotation().getRadians()) * kPYaw) * maxRotationalSpeed;


            drivetrain.setSetpoint(14.79, 4.00);

            double expectedX = drivetrain.getXSetpoint();
            double expectedY = drivetrain.getYSetpoint();
            System.out.println("\nxSpeed: " + xSpeed
            +"\nySpeed: " + ySpeed
            +"\nyawSpeed " + yawSpeed + "\ncurrentX = " + botPose.getX() + " | expectedX = " + expectedX
            + "\ncurrentY = " + botPose.getY() + " | expectedY" + expectedY);

            // reverse if a button is pressed
            if (joystick.triangle().getAsBoolean()) {
                // System.out.println("Inverting the outputs!");
                xSpeed = -xSpeed;
                ySpeed = -ySpeed;
            }

            if (drivetrain.atXSetpoint()) {
                xSpeed = 0;
            }
            if (drivetrain.atYSetpoint()) {
                ySpeed = 0;
            }

            // Don't move for now
            // return drive.withVelocityX(0) // Drive forward with negative Y (forward) (MaxSpeed)
            //     .withVelocityY(0) // Drive left with negative X (left) (MaxSpeed)
            //     .withRotationalRate(0);//-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)

            // System.out.println("Driving at those speeds!");
            return drive.withVelocityX(xSpeed * MaxSpeed) // Drive forward with negative Y (forward) (MaxSpeed)
                .withVelocityY(ySpeed * MaxSpeed) // Drive left with negative X (left) (MaxSpeed)
                .withRotationalRate(yawSpeed * MaxAngularRate)//-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                .withDeadband(0.1);
            });

        drivetrain.setDefaultCommand(teleopDriveCommmand);

        // joystick.R2().onTrue(new InstantCommand(PratheekIntake::startLoadFuel));
        // joystick.R2().onFalse(new InstantCommand(PratheekIntake::stopLoadFuel));
        
        joystick.R2().whileTrue(teleopDriveCommmand);

        joystick.L2().onTrue(new InstantCommand(() -> shooter.startLoadFuelFlywheel())
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(() -> {
                if (joystick.L2().getAsBoolean()) {
                    shooter.startLoadFuel(); 
                }
            })));
        
        // Need to set setpoints
        // GUIDE TO MAKING THE ROBOT SHOOTER SETPOINT-CONTROLLED INSTEAD OF INCREMENTAL
        // 1. change these commands (optional)
        // 2. change the periodic method of the shooter to call the PID thing command (commented out Rn)
        // joystick.povUp().onTrue(new InstantCommand(() -> shooter.setHoodSetPoint(1.0)));
        // joystick.povDown().onTrue(new InstantCommand(() -> shooter.setHoodSetPoint(0.0)));
        
        joystick.povUp().onTrue(new InstantCommand(() -> shooter.riseFuelHood()));
        joystick.povDown().onTrue(new InstantCommand(() -> shooter.lowerFuelHood()));
        joystick.povUp().onFalse(new InstantCommand(() -> shooter.stopFuelHood()));
        joystick.povDown().onFalse(new InstantCommand(() -> shooter.stopFuelHood()));

        joystick.L2().onFalse(new InstantCommand(() -> shooter.stopLoadFuel()));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.povDown().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0)//init 0.5
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
