package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Ports;

public class TurretSubsystem implements Subsystem {
    
    private final SparkMax rotatorMotor;
    private final SparkMax flywheelMotor;
    private final SparkMax lowerIntakeMotor;
    private final SparkMax upperIntakeMotor;

    private final RelativeEncoder rotatorMotorEncoder;
    private final RelativeEncoder flywheelMotorEncoder;
    private final RelativeEncoder lowerIntakeMotorEncoder;
    private final RelativeEncoder upperIntakeMotorEncoder;

    public TurretSubsystem() {

        rotatorMotor = new SparkMax(Ports.TURRET_ROTATE_PORT, MotorType.kBrushless);
        flywheelMotor = new SparkMax(Ports.TURRET_FLYWHEEL_PORT, MotorType.kBrushless);
        lowerIntakeMotor = new SparkMax(Ports.TURRET_LOWER_INTAKE_PORT, MotorType.kBrushless);
        upperIntakeMotor = new SparkMax(Ports.TURRET_UPPER_INTAKE_PORT, MotorType.kBrushless);

        rotatorMotorEncoder = rotatorMotor.getEncoder();
        flywheelMotorEncoder = flywheelMotor.getEncoder();
        lowerIntakeMotorEncoder = lowerIntakeMotor.getEncoder();
        upperIntakeMotorEncoder = upperIntakeMotor.getEncoder();
    }

    public void startRotatorMotorRight() {
        rotatorMotor.set(0.5);
    }

    public void startRotatorMotorLeft() {
        rotatorMotor.set(-0.5);
    }

    public void stopRotatorMotor() {
        rotatorMotor.set(0);
    }

    public void startLoadFuel() {
        lowerIntakeMotor.set(0.5);
        upperIntakeMotor.set(0.5);
    }

    public void stopLoadFuel() {
        lowerIntakeMotor.set(0);
        upperIntakeMotor.set(0);
    }

    public void startFiring() {
        flywheelMotor.set(0.5);
    }

    public void stopFiring() {
        flywheelMotor.set(0);
    }

    @Override
    public void periodic() {

    }
}
