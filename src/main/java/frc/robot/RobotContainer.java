// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import edu.wpi.first.wpilibj2.command.button.;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.C;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.ControlUtils;
import frc.robot.util.KinematicsUtil;

public class RobotContainer {
    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private Command teleopDriveCommand;
    private Command followApriltagCommand;
    
    private final CommandPS5Controller joystick = new CommandPS5Controller(0);
    
    public final Optional<IntakeSubsystem> intake;
    public final Optional<CommandSwerveDrivetrain> drivetrain;
    public final Optional<ShooterSubsystem> shooter;
    public final Optional<ClimberSubsystem> climber;
    public final Optional<LimelightSubsystem> limelights;
 
    public final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
    public SendableChooser<Command> autoChooser;

    private boolean shooterOn = false;
    private double controllerInversionFactor = 1;

    public static RobotContainer INSTANCE;

    public RobotContainer() {
        RobotContainer.INSTANCE = this;

        drivetrain = initializeIfAttached(Flags.DRIVETRAIN_IS_ATTACHED, TunerConstants::createDrivetrain);
        shooter = initializeIfAttached(Flags.SHOOTER_IS_ATTACHED, ShooterSubsystem::new);
        intake = initializeIfAttached(Flags.INTAKE_AND_SHOOTER_IS_ATTACHED, () -> new IntakeSubsystem(shooter.get().getIntakePivotAbsoluteEncoder()));
        climber = initializeIfAttached(Flags.CLIMBER_IS_ATTACHED, ClimberSubsystem::new);
        limelights = initializeIfAttached(Flags.DRIVETRAIN_IS_ATTACHED, () -> new LimelightSubsystem(drivetrain.get()));
        configureBindings();
        configureNamedCommands();
        configureAutoChooser();
    }

    public void configureNamedCommands() {

        // registerIfAttached("pivotOn",  intake, i -> C.instantCommand(i, intake-> intake.setTimWantsTheIntakeToMove(true)));
        // registerIfAttached("pivotOff", intake, i -> C.instantCommand(i, intake-> intake.setTimWantsTheIntakeToMove(false)));

        // Doesn't need pivotOn
        registerIfAttached("pivotDown", intake, i -> C.resourceCommand(i, intake -> {
            // initialize by enabling intake and setting setpoint to down
            intake.setTimWantsTheIntakeToMove(true);
            intake.setPivotDown();
        }, intake -> {
            // end by disabling intake
            intake.setTimWantsTheIntakeToMove(false);
        }, 1.0));
        
        registerIfAttached("intakeOn", intake, i -> C.instantCommand(i, intake -> intake.startLoadFuel()));
        registerIfAttached("intakeOff", intake, i -> C.instantCommand(i, intake -> intake.stopLoadFuel()));
        
        registerIfAttached("shoot", shooter, shooter -> new ShootCommand(shooter));

        registerIfAttached("autoAimAlongPathOn",  drivetrain, d -> C.instantCommand(d, drivetrain -> drivetrain.autoAimAlongPath = true ));
        registerIfAttached("autoAimAlongPathOff", drivetrain, d -> C.instantCommand(d, drivetrain -> drivetrain.autoAimAlongPath = false));

        registerIfAttached("aim", drivetrain, this::getStopAndAimCommand);
        
        // these are deprecated:
        // registerIfAttached("shooterLoad", shooter, s -> C.durationCommand(s, shooter -> shooter.startLoadFuel(), 1.0));
        // // registerIfAttached("pivotDownDuration", intake, i -> C.durationCommand(i, intake -> intake.setPivotDown(), 1.5));
        // registerIfAttached("pivotOffDuration", intake, i -> C.durationCommand(i, intake -> intake.setTimWantsTheIntakeToMove(false), 0.05));
        // registerIfAttached("pivotOnDuration", intake, i -> C.instantCommand(i, intake -> intake.setTimWantsTheIntakeToMove(true)));
        // registerIfAttached("intakeOnDuration", intake, i -> C.durationCommand(i, intake -> intake.startLoadFuel(), 0.1));
        // registerIfAttached("intakeOffDuration", intake, i -> C.durationCommand(i, intake -> intake.stopLoadFuel(), 0.1));
        // // double deprecated:
        // registerIfAttached("shooterAllStop", shooter, shooter -> new InstantCommand(() -> {shooter.stopLoadFuel(); shooter.setFlywheelSpeed(0.8);}));
        // registerIfAttached("printShooter", shooter, shooter -> new InstantCommand(() -> System.out.println("printShooter ran")));
        // registerIfAttached("printShooter1.0", shooter, shooter -> new InstantCommand(() -> {
        //     System.out.println("printShooter1.0 ran");
        //     shooter.setFlywheelSpeed(0.4);
        // }));
    }   

    public Pose2d getPose() {
        return drivetrain.get().getState().Pose;
    }

    public ChassisSpeeds getSpeeds() {
        return drivetrain.get().getState().Speeds;
    }

    public Command getStopAndAimShooterCommand(ShooterSubsystem shooter) {
        return new InstantCommand(shooter::shooterAutoAim);
    }

    private static <T> void registerIfAttached(String name, Optional<T> arg, Function<T, Command> getCommand) {
        if (arg.isPresent()) {
            System.out.println("registered command: " + name);
            NamedCommands.registerCommand(name, getCommand.apply(arg.get()));
        } else {
            System.out.println("Did not register un-attached subsystem: " + name);
            NamedCommands.registerCommand(name, new InstantCommand());
        }
    }

    private static <T> Optional<T> initializeIfAttached(boolean subsystemAttached, Supplier<T> newSubsystem) {
        if (subsystemAttached) {
            return Optional.of(newSubsystem.get());
        } else {
            return Optional.empty();
        }
    }

    public Pose2d getBotPose() {
        return drivetrain.map(d -> d.getState().Pose).orElse(new Pose2d());
    }

    public ChassisSpeeds getBotSpeeds() {
        return drivetrain.map(d -> d.getState().Speeds).orElse(new ChassisSpeeds());
    }

    public Command getStopAndAimCommand(CommandSwerveDrivetrain d) {
        return d.applyRequest(() -> {
            System.out.println("STOP AND AIM COMMAND EXECUTING");
            Pose2d botPose = d.getState().Pose;
            ChassisSpeeds currentSpeed = d.getState().Speeds;
            
            // System.out.println("Follow Apriltag Command");
            var shooterState = KinematicsUtil.getShooterState(botPose.getX(), botPose.getY(), currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);
            Angle yawAngle = shooterState.getSecond().getFirst();
            
            // Drive teleop, auto aiming robot yaw towards hub
            double yawSpeed = d.calculateYawSpeed(botPose.getRotation().getRadians(), yawAngle.plus(Angle.ofBaseUnits(Math.PI, Radians)).in(Radians));

            System.out.println("STOP AND AIM COMMAND DONE SOON");
            return drive.withVelocityX(0) // Drive forward with negative Y (forward) (MaxSpeed)
                .withVelocityY(0)// Drive left with negative X (left) (MaxSpeed)
                .withRotationalRate(yawSpeed * MaxAngularRate);//-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        });
    }

    private void configureAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser("Test Path");
        try {            
            // autoChooser.addOption("FRC Auto", new PathPlannerAuto("FRC Auto"));
            // autoChooser.addOption("BTTSBackAndForthTwiceBlue Auto", new PathPlannerAuto("BTTSBackAndForthTwiceBlue Auto"));
            // // autoChooser.addOption("BTTSBackAndForthTwiceRed Auto", new PathPlannerAuto("BTTSBackAndForthTwiceRed Auto"));
            // // autoChooser.addOption("MovingToClimbArea Auto", new PathPlannerAuto("MovingToClimbArea Auto"));
            // autoChooser.addOption("TTTSBackAndForthTwiceBlue Auto", new PathPlannerAuto("TTTSBackAndForthTwiceBlue Auto"));
            // autoChooser.addOption("TTTSBackAndForthTwiceRed Auto", new PathPlannerAuto("TTTSBackAndForthTwiceRed Auto"));            

            // autoChooser.addOption("BottomBlueMid", new PathPlannerAuto("BottomBlueMid"));
            // autoChooser.addOption("TopBlueMid", new PathPlannerAuto("TopBlueMid"));
            // autoChooser.addOpt/ion("CenterBlueMid", new PathPlannerAuto("CenterBlueMid"));

            
            autoChooser.addOption("ALLIANCE blueLeft", new PathPlannerAuto("ALLIANCE blueLeft"));
            autoChooser.addOption("ALLIANCE blueCenter", new PathPlannerAuto("ALLIANCE blueCenter"));
            autoChooser.addOption("ALLIANCE blueRight", new PathPlannerAuto("ALLIANCE blueRight"));

            autoChooser.addOption("MATCH1", new PathPlannerAuto("MATCH1"));
            autoChooser.addOption("MATCH2", new PathPlannerAuto("MATCH2"));
            autoChooser.addOption("OLD-MATCH2", new PathPlannerAuto("OLD-MATCH2"));

            autoChooser.addOption("Test 1", new PathPlannerAuto("Test 1"));
            autoChooser.addOption("Test 2: With New Pivot Capabilities!", new PathPlannerAuto("Test 2 With New Pivot Capabilities"));
            autoChooser.addOption("Test Pivot", new PathPlannerAuto("Test Pivot"));


            // autoChooser.addOption("BottomRedMid", new PathPlannerAuto("BottomRedMid"));
            // autoChooser.addOption("TopRedMid", new PathPlannerAuto("TopRedMid"));
            // autoChooser.addOption("CenterRedMid", new PathPlannerAuto("CenterRedMid"));

            // autoChooser.addOption("CenterBlueRace", new PathPlannerAuto("CenterBlueRace"));
            // autoChooser.addOption("SHOOT", new PathPlannerAuto("SHOOT"));


            // autoChooser.addOption("VedanthBlueLeft", new PathPlannerAuto("VedanthBlueLeft"));

            // // autoChooser.addOption("TopRedMid", new PathPlannerAuto("TopRedMid"));
            // autoChooser.addOption("Shooter Test", new PathPlannerAuto("Shooter Test"));
            // autoChooser.addOption("NothingAuto", new PathPlannerAuto("NothingAuto"));


            // autoChooser.setDefaultOption("FRC Auto", new PathPlannerAuto("FRC Auto"));
        } catch (Exception e) {
            DriverStation.reportError("we're dumb: " + e.getMessage(), e.getStackTrace());
        }
        SmartDashboard.putData("Auto Choices 2026", autoChooser);
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention
        pdp.setSwitchableChannel(true);

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            controllerInversionFactor *= -1; // TEST ME!
        }


            
        drivetrain.ifPresent(drivetrain -> {                                                                                                                                                                                                                                                                                              
            drivetrain.initPID();         
            
            drivetrain.seedFieldCentric();
            // drivetrain.getPigeon2().setYaw(0);

            joystick.options().onTrue(new InstantCommand(() -> {
                controllerInversionFactor *= -1;
            }));

            joystick.PS().onTrue(new InstantCommand(() -> {
                drivetrain.seedFieldCentric();
                // drivetrain.getPigeon2().setYaw(0);
            }));

            teleopDriveCommand = drivetrain.applyRequest(() -> {
                return drive.withVelocityX(controllerInversionFactor * ControlUtils.smoothJoystick(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward) (MaxSpeed)
                            .withVelocityY(controllerInversionFactor * ControlUtils.smoothJoystick(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left) (MaxSpeed)
                            .withRotationalRate(-joystick.getRightX()*MaxAngularRate) // Drive counterclockwise with negative X (left)
                            .withDeadband(0.1);
            });

            followApriltagCommand = drivetrain.applyRequest(() -> {
                // System.out.println("\n\n\nfollow apriltag command");
                Pose2d botPose = drivetrain.getState().Pose;
                ChassisSpeeds currentSpeed = drivetrain.getState().Speeds;
                
                // System.out.println("Follow Apriltag Command");
                var shooterState = KinematicsUtil.getShooterState(botPose.getX(), botPose.getY(), currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);
                if (botPose.getX() > 4.6256 && botPose.getX() < 11.9154) { // 182.11 143.5
                    shooterState = KinematicsUtil.getShooterStateForPass(botPose.getX(), botPose.getY(), currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);
                }
                LinearVelocity flywheelVelocty = shooterState.getFirst();
                Angle yawAngle = shooterState.getSecond().getFirst();
                Angle hoodAngle = shooterState.getSecond().getSecond();
                
                shooter.ifPresent(shooter -> {
                    // Auto aim shooter
                    // System.out.println("hood setpoint = " + hoodAngle);
                    shooter.setHoodSetpoint(hoodAngle);
                    // System.out.println(drivetrain.atYawSetpoint());
                    shooter.setFlywheelSpeed(flywheelVelocty);
                });
                // Drive teleop, auto aiming robot yaw towards hub
                double yawSpeed = drivetrain.calculateYawSpeed(botPose.getRotation().getRadians(), yawAngle.in(Radians));
                yawSpeed = ControlUtils.clamp(-0.9, yawSpeed, 0.9);

                return drive.withVelocityX(controllerInversionFactor * ControlUtils.smoothJoystick(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward) (MaxSpeed)
                    .withVelocityY(controllerInversionFactor * ControlUtils.smoothJoystick(joystick.getLeftX()) * MaxSpeed)// Drive left with negative X (left) (MaxSpeed)
                    .withRotationalRate(yawSpeed * MaxAngularRate)//-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                    .withDeadband(0.1);
            });
            
            drivetrain.setDefaultCommand(teleopDriveCommand);
        
            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
            final var idle = new SwerveRequest.Idle();
            RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
            );
            
            // joystick.PS().onTrue(new InstantCommand(() -> {
            //     LimelightHelpers.SetIMUMode("limelight-a", 1);
            //     LimelightHelpers.SetIMUMode("limelight-b", 1);
            //     LimelightHelpers.SetIMUMode("limelight-c", 1);
            //     drivetrain.runOnce(drivetrain::seedFieldCentric).execute();
            //     limelights.ifPresent(limelights -> limelights.updateMegaTag2RobotYaw());
            // }).finallyDo(() -> {
            //     LimelightHelpers.SetIMUMode("limelight-a", 4);
            //     LimelightHelpers.SetIMUMode("limelight-b", 4);
            //     LimelightHelpers.SetIMUMode("limelight-c", 4);
            // }));
            drivetrain.registerTelemetry(logger::telemeterize);         
        });

        shooter.ifPresent(shooter -> {
            joystick.triangle().onTrue(new InstantCommand(() -> shooter.setHoodSetpoint(1.0)));
            joystick.circle().onTrue(new InstantCommand(() -> shooter.setHoodSetpoint(0.0)));
            // joystick.triangle().onTrue(new InstantCommand(() -> intake.pivotTowardPositive()));
            // joystick.triangle().onFalse(new InstantCommand(intake::pivotStop));
            // joystick.cross().onTrue(new InstantCommand(() -> intake.pivotTowardNegative()));
            // joystick.cross().onFalse(new InstantCommand(intake::pivotStop));

            // cross to disable auto aim,
            // if (joystick.R1().getAsBoolean() == false) {
            joystick.R2().and(joystick.R1().negate()).whileTrue(followApriltagCommand);
            // }
            // then R2 to shoot (waits for flywheel to get up to speed)
            joystick.R2().onTrue(
                new InstantCommand(() -> {
                    // TODO: do we alwayd auto aim now?
                    if (joystick.R1().getAsBoolean() == true) {
                        shooter.setFlywheelSpeed(0.8); // done twice check follow apriltag command
                    } else {
                        shooterOn = true;
                    } 
                })
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(() -> {
                    if (joystick.R2().getAsBoolean() == true && joystick.R1().getAsBoolean()) {
                        shooter.startLoadFuel();
                    }
                }).repeatedly().onlyWhile(() -> joystick.R2().getAsBoolean()))
            );
            joystick.R2().onFalse(new InstantCommand(() -> {
                shooterOn = false;
                shooter.stopLoadFuel();
            }));

            joystick.R2().whileTrue(
                new WaitUntilCommand(() -> drivetrain.get().atYawSetpoint())
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(shooter::startLoadFuel))
            );
        });
        
        intake.ifPresent(intake -> {
            // L1 to toggle the pivot up and down
            // Note: now only toggles it to down, doesn't go up
            joystick.L1().onTrue(new InstantCommand(intake::togglePivot));

            joystick.circle().onTrue(new InstantCommand(intake::setPivotUp));

            // joystick.circle().whileTrue(intake::set)

            // joystick.povUp().onTrue(new InstantCommand(intake::pivotTowardPositive));
            // joystick.povDown().onTrue(new InstantCommand(intake::pivotTowardNegative));
            // joystick.povUp().onFalse(new InstantCommand(intake::pivotStop));
            // joystick.povDown().onFalse(new InstantCommand(intake::pivotStop));

            // joystick.povUp().onTrue(new InstantCommand(intake::startPivotUpwards));
            // joystick.povUp().onFalse(new InstantCommand(intake::stopPivot));
            // joystick.povDown().onTrue(new InstantCommand(intake::startPivotDown));
            // joystick.povDown().onFalse(new InstantCommand(intake::stopPivot));

            // hold L2 to intake/not take fuel
            joystick.L2().onTrue(new InstantCommand(intake::startLoadFuel));
            joystick.L2().onFalse(new InstantCommand(intake::stopLoadFuel));
            joystick.triangle().onTrue(new InstantCommand(intake::startSpitFuel));
            joystick.triangle().onFalse(new InstantCommand(intake::stopLoadFuel));
        
            joystick.square().onTrue(new InstantCommand(() -> intake.setTimWantsTheIntakeToMove(true)));
            joystick.square().onFalse(new InstantCommand(() -> intake.setTimWantsTheIntakeToMove(false)));

            // // jiggle that thing while shooting (r2)
            // joystick.R2().whileTrue(new WaitCommand(1)
            //     .andThen(new RepeatCommand(
            //         new InstantCommand(intake::setPivotMiddle)
            //         .andThen(new WaitCommand(0.5))
            //         .andThen(new InstantCommand(intake::setPivotJiggle))
            //         .andThen(new WaitCommand(0.5))
            // )));
            // joystick.R2().onFalse(new InstantCommand(intake::setPivotUp));
        });
            // luca did ALL of this ^^^
        climber.ifPresent(climber -> {
            // joystick.create(); left weird button
            // joystick.touchpad();
            // joystick.PS();
            // joystick.options(); right weird button
            // joystick.R1().onTrue(climber::toggleHopper);
        });

        // ------------------------------------------------------------------------------------------------------
        //                           End of our code. Template code below:
        // ------------------------------------------------------------------------------------------------------

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
    }
    
    public Command getAutonomousCommand() {
        // drivetrain.get().seedFieldCentric();
        return autoChooser.getSelected();
        // return null;

        // ------------------------------------------------------------------------------------------------------
        //                           End of our code. Template code below:
        // ------------------------------------------------------------------------------------------------------
        
        // TODO: remove this template code once real autons work.
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
