package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax lowerIntakeMotor;
    private final SparkFlex roller;
    private final SparkFlex agitator;
    private final SparkMax flywheel;
    private final SparkMax upperIntakeMotor;
    private final SparkMax hoodCover;

    private final AbsoluteEncoder hoodCoverAbsoluteEncoder;
    private final double hoodZeroPosition = 0.1448;
    private final double hoodOnePosition = 0.0483;
    private final double hoodPositionRange = 1 - hoodZeroPosition + hoodOnePosition;

    private final PIDController hoodPositionController = new PIDController(0.5, 0, 0);

    public ShooterSubsystem() {
        lowerIntakeMotor = new SparkMax(Ports.LOWER_INTAKE_CAN_ID, MotorType.kBrushless);
        roller = new SparkFlex(Ports.ROLLER_CAN_ID, MotorType.kBrushless);
        agitator = new SparkFlex(Ports.AGITATOR_CAN_ID, MotorType.kBrushless);
        flywheel = new SparkMax(Ports.FLYWHEEL_CAN_ID, MotorType.kBrushless);
        upperIntakeMotor = new SparkMax(Ports.UPPER_INTAKE_CAN_ID, MotorType.kBrushless);
        hoodCover = new SparkMax(Ports.HOOD_COVER_CAN_ID, MotorType.kBrushless);

        hoodCoverAbsoluteEncoder = hoodCover.getAbsoluteEncoder();
        hoodPositionController.setSetpoint(0.0);
    }

    public void startLoadFuelFlywheel() {
        flywheel.set(-0.75);
    }
    public void setFlywheelSpeed(double speed) {
        flywheel.set(speed);
    }
    public void riseFuelHood() { // Need to check whether changing the position of the hood will collide with the min/max thresholds
        hoodCover.set(-0.2);
    }
    public void lowerFuelHood() {
        hoodCover.set(0.2);
        
    }
    public void stopFuelHood() {
        hoodCover.set(0);
    }
    public void setHoodSetPoint(double setPoint) {
        hoodPositionController.setSetpoint(setPoint);
    }

    public double getHoodPosition() {
        double rawPosition = hoodCoverAbsoluteEncoder.getPosition();
        double possiblyNegativeTruePosition = rawPosition - hoodZeroPosition;
        if (possiblyNegativeTruePosition < 0) {
            possiblyNegativeTruePosition += 1;
        }
        double normalizedPosition = possiblyNegativeTruePosition / hoodPositionRange;
        return normalizedPosition;
    }
    
    public void runHoodPositionPID() {
        hoodCover.set(-hoodPositionController.calculate(getHoodPosition()));
    }

    public void startLoadFuel() {
        lowerIntakeMotor.set(-0.6);
        roller.set(0.6);
        agitator.set(-0.6);
        upperIntakeMotor.set(0.6);
    }

    public void stopLoadFuel() {
        lowerIntakeMotor.set(0);
        roller.set(0);
        agitator.set(0);
        flywheel.set(0);
        upperIntakeMotor.set(0);    
    }

    @Override
    public void periodic() {
        // Check to see if the limit switch has been triggered
        // System.out.println("hood position = " + hoodCoverAbsoluteEncoder.getPosition());
        runHoodPositionPID();
    }
}