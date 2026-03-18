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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import edu.wpi.first.wpilibj2.command.button.;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.ControlUtils;
import frc.robot.util.KinematicsUtil;
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
    public final ClimberSubsystem climber = new ClimberSubsystem();
    public final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
    private boolean shooterOn = false;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Test Path");
        try {
            PathPlannerPath testPath = PathPlannerPath.fromPathFile("Test Path");
            PathPlannerAuto FRCAuto = new PathPlannerAuto("FRC Auto");
            Command simplePath = AutoBuilder.followPath(testPath);
            
            autoChooser.setDefaultOption("Path 1", simplePath);
            autoChooser.addOption("Path 1", simplePath);
            autoChooser.addOption("FRC Auto", FRCAuto);
        } catch (Exception e) {
            DriverStation.reportError("we're dumb: " + e.getMessage(), e.getStackTrace());
        }
        SmartDashboard.putData("Auto Choices 2026", autoChooser);

        configureBindings();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Initialize limelights and drivetrain yaw for odometry
        LimelightHelpers.SetIMUMode("limelight-a", 1);
        LimelightHelpers.SetIMUMode("limelight-b", 1);
        LimelightHelpers.SetIMUMode("limelight-c", 1);
        limelights.updateMegaTag2RobotYaw();
        drivetrain.initPID();
        pdp.setSwitchableChannel(true);

        teleopDriveCommmand = drivetrain.applyRequest(() -> {
            return drive.withVelocityX(Math.pow(joystick.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward) (MaxSpeed)
                        .withVelocityY(Math.pow(joystick.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left) (MaxSpeed)
                        .withRotationalRate(-joystick.getRightX()*MaxAngularRate) // Drive counterclockwise with negative X (left)
                        .withDeadband(0.1);
        });
    
        followApriltagCommand = drivetrain.applyRequest(() -> {
            Pose2d botPose = drivetrain.getState().Pose;
            // Pose2d botPose = new Pose2d(13.9150, 4.0345, new Rotation2d(0));

            ChassisSpeeds currentSpeed = drivetrain.getState().Speeds;
            
            // System.out.println("Follow Apriltag Command");
            var shooterState = KinematicsUtil.getShooterState(botPose.getX(), botPose.getY(), currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);
            LinearVelocity flywheelVelocty = shooterState.getFirst();
            Angle yawAngle = shooterState.getSecond().getFirst();
            Angle hoodAngle = shooterState.getSecond().getSecond();
            
            // Auto aim shooter
            shooter.setHoodSetpoint(hoodAngle);
            if (shooterOn) {
                shooter.setFlywheelSpeed(flywheelVelocty);
            } else {
                shooter.setFlywheelSpeed(0);
            }
            
            // Drive teleop, auto aiming robot yaw towards hub
            double rotationSpeedLimiter = 0.2;
            double yawSpeed = drivetrain.calculateYawSpeed(botPose.getRotation().getRadians(), yawAngle.in(Radians));
            yawSpeed = ControlUtils.clamp(yawSpeed) * rotationSpeedLimiter; // probably change how this works later

            return drive.withVelocityX(Math.pow(joystick.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward) (MaxSpeed)
                .withVelocityY(Math.pow(joystick.getLeftX(), 3) * MaxSpeed)// Drive left with negative X (left) (MaxSpeed)
                .withRotationalRate(yawSpeed * MaxAngularRate)//-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                .withDeadband(0.1);
        });

        drivetrain.setDefaultCommand(teleopDriveCommmand);        
        
        // R2 to auto aim,
        joystick.R2().whileTrue(followApriltagCommand);
        // then L2 to shoot, only during auto aim (waits for flywheel to get up to speed)
        joystick.L2().onTrue(
            new InstantCommand(() -> { 
                if (joystick.R2().getAsBoolean() == true) {
                    shooterOn = true;
                } else {
                    shooter.setFlywheelSpeed(0.6);
                } 
            })
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(() -> {
                if (joystick.L2().getAsBoolean() == true) {
                    shooter.startLoadFuel(); 
                }
            }))
        );
        joystick.L2().onFalse(new InstantCommand(() -> {
            shooterOn = false;
            shooter.stopLoadFuel();
        }));

        // // Non-auto aim with cross
        // joystick.cross().onTrue(
        //     new InstantCommand(() -> { shooter.setFlywheelSpeed(0.6); })
        //     .andThen(new WaitCommand(0.8))
        //     .andThen(new InstantCommand(() -> {
        //         if (joystick.cross().getAsBoolean() == true) {
        //             shooter.startLoadFuel();
        //         }
        //     }))
        // );
        // joystick.cross().onFalse(new InstantCommand(() -> {
        //     shooter.stopLoadFuel();
        //     shooter.setFlywheelSpeed(0);
        // }));
        
        // GUIDE TO MAKING THE ROBOT SHOOTER SETPOINT-CONTROLLED INSTEAD OF INCREMENTAL
        // 1. change these commands (optional)
        // 2. change the periodic method of the shooter to call the PID thing command (commented out Rn)
        // joystick.povUp().onTrue(new InstantCommand(() -> shooter.setHoodSetPoint(1.0)));
        // joystick.povDown().onTrue(new InstantCommand(() -> shooter.setHoodSetPoint(0.0)));
        // TODO: intake pivot / 27979
        // joystick.povUp().onTrue(new InstantCommand(intake));
        // joystick.povDown().onTrue(new InstantCommand(shooter::lowerFuelHood));
        // joystick.povUp().onFalse(new InstantCommand(shooter::stopFuelHood));
        // joystick.povDown().onFalse(new InstantCommand(shooter::stopFuelHood));


        // disabled for now while tim's thing is attached to it.
        // joystick.povUp().onTrue(new InstantCommand(intake::startPivotUpwards));
        // joystick.povDown().onTrue(new InstantCommand(intake::startPivotDown));
        // joystick.povUp().onFalse(new InstantCommand(intake::stopPivot));
        // joystick.povDown().onFalse(new InstantCommand(intake::stopPivot));

        // L1 to intake fuel
        // joystick.L1().onTrue(new InstantCommand(intake::startLoadFuel));
        // joystick.L1().onFalse(new InstantCommand(intake::stopLoadFuel));

        // End of our code. Template code below:
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
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
        // Note: For some reason this doesn't work for us? 
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Fix for gyro starting backwards problem, hopefully
        // if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
        //     drivetrain.seedFieldCentric(Rotation2d.k180deg);
        // } else {
        //     drivetrain.seedFieldCentric(Rotation2d.kZero);
        // }

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

        // // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> {
        //         System.out.println("Auto centered!");
        //         drivetrain.seedFieldCentric(Rotation2d.kZero); // Does this account for which team we are? I guess it must
        //     }),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0)//init 0.5
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
    }
}
