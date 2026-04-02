package frc.robot;

        // intake = initializeIfAttached(Flags.INTAKE_IS_ATTACHED, IntakeSubsystem::new);
        // drivetrain = initializeIfAttached(Flags.DRIVETRAIN_IS_ATTACHED, TunerConstants::createDrivetrain);
        // shooter = initializeIfAttached(Flags.SHOOTER_IS_ATTACHED, ShooterSubsystem::new);
        // climber = initializeIfAttached(Flags.CLIMBER_IS_ATTACHED, ClimberSubsystem::new);
        // limelights = initializeIfAttached(Flags.DRIVETRAIN_IS_ATTACHED, () -> new LimelightSubsystem(drivetrain.get()));


public class Flags {
    // Intake uses the shooter TSA motor's absolute encoder so it depends on shooter.
    public static boolean INTAKE_AND_SHOOTER_IS_ATTACHED = true;
    public static boolean DRIVETRAIN_IS_ATTACHED = true;
    public static boolean SHOOTER_IS_ATTACHED = true;
    public static boolean CLIMBER_IS_ATTACHED = false;
}
