// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.subsystems.IntakeSubsystem;
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
    private Command pointAndShootCommand;
    
    // private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandPS5Controller joystick = new CommandPS5Controller(0);
    
    public final IntakeSubsystem intake = new IntakeSubsystem();
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
        limelights.updateMegaTag2RobotYaw();

        teleopDriveCommmand = drivetrain.applyRequest(() -> {
            System.out.println("teleopDrive");
            return drive.withVelocityX(Math.pow(joystick.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward) (MaxSpeed)
                .withVelocityY(Math.pow(joystick.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left) (MaxSpeed)
                .withRotationalRate(-joystick.getRightX()*MaxAngularRate) // Drive counterclockwise with negative X (left)
                .withDeadband(0.1);
        });
    


        followApriltagCommand = drivetrain.applyRequest(() -> {
            Pose2d botPose = drivetrain.getState().Pose;

            double maxSpeed = 0.1;

            double maxRotationalSpeed = 0.2;

            double hubX = Constants.FieldConstants.RED_HUB_CENTER_TRANSLATION.getX();
            double hubY = Constants.FieldConstants.RED_HUB_CENTER_TRANSLATION.getY();

            double xSpeed = Math.pow(joystick.getLeftY(), 3) * MaxSpeed;
            double ySpeed = Math.pow(joystick.getLeftX(), 3) * MaxSpeed;
            double xDifference = hubX - botPose.getX();
            double yDifference = hubY - botPose.getY();
            double hubYaw = Math.atan(yDifference / xDifference); // clockwise is positive
            double hubDifference = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(xDifference, 2));
            double fuelSpeed = ControlUtils.getFuelSpeed(hubDifference);
            double expectedYaw = Math.atan((fuelSpeed*Math.sin(hubYaw) - ySpeed)/(fuelSpeed*Math.cos(hubYaw) - xSpeed));
            // roughly -pi/2
            if (xDifference < 0) {
                expectedYaw += Math.PI;
            }
            System.out.println("expectedYaw = " + expectedYaw + 
                "\nxPosition = " + drivetrain.getState().Pose.getX() + 
                "\nyPosition = " + drivetrain.getState().Pose.getY());
            // 

            // expectedYaw += Math.PI / 2;
            // // face backwards so limelight can still work
            // expectedYaw += 0.38; // Correction factor: true zero is off by 0.38.
            // expectedYaw += Math.PI; // correct for using wpiBlue instead of red
            while (expectedYaw > Math.PI) {
                expectedYaw -= Math.PI * 2;
            }
            while (expectedYaw < -Math.PI) {
                expectedYaw += Math.PI * 2;
            }

            double yawSpeed = ControlUtils.clamp(drivetrain.calculateYawSpeed(botPose.getRotation().getRadians(), expectedYaw)) * maxRotationalSpeed;

            // System.out.println(
            //     "expectedYaw=" + expectedYaw + " | currentYaw = " + botPose.getRotation() +  " | yawSpeed = " + yawSpeed + 
            //     "\nxDifference= " + xDifference + " | robotX=" + botPose.getX() + " | hubX=" + hubX +
            //     "\nyDifference= " + yDifference + " | robotY=" + botPose.getY() + " | hubY=" + hubY
            // );


            drivetrain.setSetpoint(14.79, 4.00);
            // System.out.println("hood position (unclamped) = " + ControlUtils.clamp(yawSpeed));
            System.out.println("flywheel speed" + ControlUtils.getFlywheelSpeed(drivetrain.distanceToHub()) + 
              "\ndistanceToHub = " + drivetrain.distanceToHub() + 
             "\nhood setpoint (raw) = " + ControlUtils.getRawHoodPosition(drivetrain.distanceToHub()));

            
            double hoodPosition = ControlUtils.getHoodPosition(drivetrain.distanceToHub());
            shooter.setHoodSetPoint(hoodPosition);

            // System.out.println("Driving at those speeds!");
            return drive.withVelocityX(Math.pow(joystick.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward) (MaxSpeed)
                .withVelocityY(Math.pow(joystick.getLeftX(), 3) * MaxSpeed)// Drive left with negative X (left) (MaxSpeed)
                .withRotationalRate(yawSpeed * MaxAngularRate)//-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                .withDeadband(0.1);
        });

        

        drivetrain.setDefaultCommand(followApriltagCommand);

        // joystick.R2().onTrue(new InstantCommand(PratheekIntake::startLoadFuel));
        // joystick.R2().onFalse(new InstantCommand(PratheekIntake::stopLoadFuel));
        
        joystick.R2().whileTrue(teleopDriveCommmand);

        joystick.L2().onTrue(new InstantCommand(() -> {
            double distanceToHub = drivetrain.distanceToHub();
            double flywheelSpeed = ControlUtils.getFlywheelSpeed(distanceToHub);
            shooter.setFlywheelSpeed(flywheelSpeed);
        })
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(() -> {
                if (joystick.L2().getAsBoolean()) {
                    shooter.startLoadFuel(); 
                }
            })));
        joystick.L2().onFalse(new InstantCommand(() -> shooter.stopLoadFuel()));
        
        // Need to set setpoints
        // GUIDE TO MAKING THE ROBOT SHOOTER SETPOINT-CONTROLLED INSTEAD OF INCREMENTAL
        // 1. change these commands (optional)
        // 2. change the periodic method of the shooter to call the PID thing command (commented out Rn)
        // joystick.povUp().onTrue(new InstantCommand(() -> shooter.setHoodSetPoint(1.0)));
        // joystick.povDown().onTrue(new InstantCommand(() -> shooter.setHoodSetPoint(0.0)));
        

        joystick.circle().onTrue(new InstantCommand(intake::startLoadFuel));
        joystick.circle().onFalse(new InstantCommand(intake::stopLoadFuel));

        // joystick.povUp().onTrue(new InstantCommand(shooter::riseFuelHood));
        // joystick.povDown().onTrue(new InstantCommand(shooter::lowerFuelHood));
        // joystick.povUp().onFalse(new InstantCommand(shooter::stopFuelHood));
        // joystick.povDown().onFalse(new InstantCommand(shooter::stopFuelHood));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.circle().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.povDown().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Fix for gyro starting backwards problem
        if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            drivetrain.seedFieldCentric(Rotation2d.k180deg);
        } else {
            drivetrain.seedFieldCentric(Rotation2d.kZero);
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> {
                System.out.println("Auto centered!");
                drivetrain.seedFieldCentric(Rotation2d.kZero);
            }),
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
