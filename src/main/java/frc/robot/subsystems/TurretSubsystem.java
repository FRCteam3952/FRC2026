package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
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
    private final PIDController rotatorPIDController;
    private double rotatorTargetAngle;
    
    private int rotationsSmallGear;
    private boolean rotatorInitialized;

    private double previousRotatorPosition;
    private boolean previousLimitSwitchValue;


    public TurretSubsystem() {
        rotatorMotor = new SparkMax(Ports.TURRET_ROTATE_PORT, MotorType.kBrushless);
        flywheelMotor = new SparkMax(Ports.TURRET_FLYWHEEL_PORT, MotorType.kBrushless);
        lowerIntakeMotor = new SparkMax(Ports.TURRET_LOWER_INTAKE_PORT, MotorType.kBrushless);
        upperIntakeMotor = new SparkMax(Ports.TURRET_UPPER_INTAKE_PORT, MotorType.kBrushless);
        limitSwitch = new DigitalInput(0);
        
        rotatorMotorEncoder = rotatorMotor.getEncoder();
        flywheelMotorEncoder = flywheelMotor.getEncoder();
        lowerIntakeMotorEncoder = lowerIntakeMotor.getEncoder();
        upperIntakeMotorEncoder = upperIntakeMotor.getEncoder();

        rotatorMotorAbsoluteEncoder = rotatorMotor.getAbsoluteEncoder();
        rotatorPIDController = new PIDController(0.2, 0, 0.05); // TODO: tune
        rotatorPIDController.setTolerance(0.01); // TODO: tune

        rotatorTargetAngle = 0; // TODO: change this to move towards the limit switch?

        rotatorInitialized = false;
        rotationsSmallGear = 0;

        previousRotatorPosition = rotatorMotorAbsoluteEncoder.getPosition();
        previousLimitSwitchValue = false; // this has to be false so it can reset at the beginning
    }

    public boolean atRotatorLimit() {
        return this.limitSwitch.get();
    }

    public void startRotatorMotorRight() {
        // TODO: Should this logic be in startRotatorMotorRight or startRotatorMotorLeft?
        if (limitSwitch.get() == false) {
            rotatorMotor.set(0.1);
        }
    }

    public void startRotatorMotorLeft() {
        if (limitSwitch.get() == false) {
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
        // TODO: flywheel PID control?
        flywheelMotor.set(0.5);
    }

    public void stopFiring() {
        flywheelMotor.set(0);
    }

    public Optional<Rotation2d> tryGetRotatorAngle() {
        if (!rotatorInitialized) {
            return Optional.empty();
        }
        // One small gear rotation is 1/4 of one big gear rotation.
        // 0.0 <= exactRotations < 4.0
        double exactRotations = rotationsSmallGear + rotatorMotorEncoder.getPosition();
        return Optional.of(Rotation2d.fromDegrees(exactRotations * 90));
    }

    public void setRotatorAngle(Rotation2d newTarget) {
        this.rotatorTargetAngle = newTarget.getRotations();
    }

    public boolean getAtSetpoint() {
        return this.rotatorPIDController.atSetpoint();
    }

    @Override
    public void periodic() {
        // Check to see if the limit switch has been triggered
        if (limitSwitch.get() && previousLimitSwitchValue == false) { 
            rotationsSmallGear = 0;
            rotatorMotorEncoder.setPosition(0);
            rotatorMotor.stopMotor();
            rotatorInitialized = true;
        }

        // We finished a rotation, overflowing from a high value like 0.9 back down to 0.1ish
        if (previousRotatorPosition > 0.8 && rotatorMotorEncoder.getPosition() < 0.2) {
            rotationsSmallGear++;
        }
        // Opposite case
        if (previousRotatorPosition < 0.2 && rotatorMotorEncoder.getPosition() > 0.8) {
            rotationsSmallGear--;
        }

        // Update the previous values
        previousLimitSwitchValue = limitSwitch.get();
        previousRotatorPosition = rotatorMotorEncoder.getPosition();

        if (rotatorInitialized) {
            rotatorPIDController.setSetpoint(rotatorTargetAngle);
            rotatorMotor.set(rotatorPIDController.calculate(tryGetRotatorAngle().get().getRotations()));
        } else {
            // TODO: move rotator towards limit switch here, whichever direction that is
        }
    }
}
