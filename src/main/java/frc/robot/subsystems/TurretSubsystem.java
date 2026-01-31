package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Ports;

public class TurretSubsystem implements Subsystem {
    
    private final SparkMax rotatorMotor;
    private final SparkMax flywheelMotor;
    private final SparkMax lowerIntakeMotor;
    private final SparkMax upperIntakeMotor;
    private final DigitalInput limitSwitch;

    private final RelativeEncoder rotatorMotorEncoder;
    private final RelativeEncoder flywheelMotorEncoder;
    private final RelativeEncoder lowerIntakeMotorEncoder;
    private final RelativeEncoder upperIntakeMotorEncoder;

    private final AbsoluteEncoder rotatorMotorAbsoluteEncoder;
    private double AbsoluteEncoderPos;

    private double rotationsSmallGear;
    private double rotationsBigGear;

    public TurretSubsystem() {

        rotatorMotor = new SparkMax(Ports.TURRET_ROTATE_PORT, MotorType.kBrushless);
        flywheelMotor = new SparkMax(Ports.TURRET_FLYWHEEL_PORT, MotorType.kBrushless);
        lowerIntakeMotor = new SparkMax(Ports.TURRET_LOWER_INTAKE_PORT, MotorType.kBrushless);
        upperIntakeMotor = new SparkMax(Ports.TURRET_UPPER_INTAKE_PORT, MotorType.kBrushless);
        limitSwitch = new DigitalInput(0);
        
        rotatorMotorEncoder = rotatorMotor.getEncoder();
        rotatorMotorAbsoluteEncoder = rotatorMotor.getAbsoluteEncoder();
        flywheelMotorEncoder = flywheelMotor.getEncoder();
        lowerIntakeMotorEncoder = lowerIntakeMotor.getEncoder();
        upperIntakeMotorEncoder = upperIntakeMotor.getEncoder();

        AbsoluteEncoderPos = rotatorMotorAbsoluteEncoder.getPosition();
    }

    public void startRotatorMotorRight() {
        if (limitSwitch.getChannel() == 1) {
            rotatorMotor.set(0.1);
        }
    }

    public void startRotatorMotorLeft() {
        if (limitSwitch.getChannel() == 1) {
            rotatorMotor.set(-0.1);
        } 
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

    public void convertToLargeGearRotations() {
        // We keep track of the small gear's rotations through the absolute encoder,
        // but this is not the same as the rotations of the big gear's rotations (due
        // to the difference in size)

        rotationsBigGear = rotationsSmallGear * 4;
    }

    @Override
    public void periodic() {
        if (limitSwitch.get()) { // Check to see if the limit switch has been triggered
            rotationsSmallGear = 0;
            rotatorMotor.stopMotor();
        }
        if (AbsoluteEncoderPos > 0.9 && rotatorMotorAbsoluteEncoder.getPosition() < 0.2) {
            rotationsSmallGear++;
        }
        if (AbsoluteEncoderPos < 0.2 && rotatorMotorAbsoluteEncoder.getPosition() > 0.9) {
            rotationsSmallGear--;
        }
    }
}
