package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.concurrent.atomic.DoubleAccumulator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.util.ControlUtils;
import frc.robot.util.KinematicsUtil;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax lowerIntakeMotor;
    public final SparkMax roller;
    private final SparkFlex agitator;
    private final SparkFlex flywheel;
    private final SparkFlex flywheel2;
    private final SparkMax upperIntakeMotor;
    private final TalonFX hoodCover;

    private final AbsoluteEncoder hoodCoverAbsoluteEncoder;
    private final double hoodZeroPosition = 0.0700; //down .78
    private final double hoodOnePosition = 0.7200; //up .52
    private final double hoodPositionRange = hoodOnePosition - hoodZeroPosition;

    private final PIDController hoodPositionController = new PIDController(0.5, 0, 0);

    public ShooterSubsystem() {
        lowerIntakeMotor = new SparkMax(Ports.LOWER_INTAKE_CAN_ID, MotorType.kBrushless);
        roller = new SparkMax(Ports.ROLLER_CAN_ID, MotorType.kBrushless);
        agitator = new SparkFlex(Ports.AGITATOR_CAN_ID, MotorType.kBrushless);
        flywheel = new SparkFlex(Ports.FLYWHEEL_CAN_ID, MotorType.kBrushless);
        flywheel2 = new SparkFlex(Ports.FLYWHEEL2_CAN_ID, MotorType.kBrushless);
        upperIntakeMotor = new SparkMax(Ports.UPPER_INTAKE_CAN_ID, MotorType.kBrushless);
        hoodCover = new TalonFX(Ports.HOOD_COVER_CAN_ID);

        hoodCoverAbsoluteEncoder = lowerIntakeMotor.getAbsoluteEncoder();
        hoodPositionController.setSetpoint(0.0);
        
        // SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        // flywheelConfig.voltageCompensation(9.0);
        // flywheel.configure(flywheelConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        // flywheel2.configure(flywheelConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void startLoadFuelFlywheel() {
        setFlywheelSpeed(-0.7);
    }

    public void setFlywheelSpeed(double speed) {
        // System.out.println("speed = " + speed);
        flywheel.set(speed);
        flywheel2.set(-speed);
    }

    public void stopFlywheel() {
        setFlywheelSpeed(0);
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

    public void setHoodSetpoint(double setPoint) {
        setPoint = ControlUtils.clamp(0.0, setPoint, 1.0);
        hoodPositionController.setSetpoint(setPoint);
    }

    public void setHoodSetpoint(Angle angleFromHorizontal) {
        double degreesFromVertical = 90 - angleFromHorizontal.in(Degrees); 
        double shooterRange = Constants.RobotConstants.maxShooterAngle - Constants.RobotConstants.minShooterAngle;
        double setpoint = (degreesFromVertical - Constants.RobotConstants.minShooterAngle) / shooterRange;
        setHoodSetpoint(setpoint);
    }

    public double printHoodSetpoint(Angle angleFromHorizontal) {
        double degreesFromVertical = 90 - angleFromHorizontal.in(Degrees); 
        double shooterRange = Constants.RobotConstants.maxShooterAngle - Constants.RobotConstants.minShooterAngle;
        double setpoint = (degreesFromVertical - Constants.RobotConstants.minShooterAngle) / shooterRange;
        return setpoint;
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
        // TEMPORARY DISABLE
        // System.out.println("getHoodPosition() = " + getHoodPosition());
        hoodCover.set(-hoodPositionController.calculate(getHoodPosition()));
    }
    
    public void setFlywheelSpeed(LinearVelocity flywheelVelocity) {
        double fuelSpeed = flywheelVelocity.in(MetersPerSecond);
        double fuelSpeedToMotorOutput = 0.03131625;
        double flywheelSpeedLossFactor = 4; // previously 1.3

        double motorOutput = fuelSpeed * fuelSpeedToMotorOutput * flywheelSpeedLossFactor;
        // System.out.println("motorOutput: " + motorOutput);
        setFlywheelSpeed(ControlUtils.clamp(0, motorOutput, 1));
    }

    public void startLoadFuel() {
        lowerIntakeMotor.set(0.8);
        roller.set(0.9);
        agitator.set(0.6);
        upperIntakeMotor.set(0.8);
    }

    public void stopLoadFuel() {
        lowerIntakeMotor.set(0);
        roller.set(0);
        agitator.set(0);
        flywheel.set(0);
        flywheel2.set(0);
        upperIntakeMotor.set(0);    
    }

    @Override
    public void periodic() {
        // Check to see if the limit switch has been triggered
        System.out.println("hood position = " + hoodCoverAbsoluteEncoder.getPosition());
        runHoodPositionPID();
        // System.out.println("up = " + printHoodSetpoint(Angle.ofBaseUnits(45, Degrees)));
        // System.out.println("down = " + printHoodSetpoint(Angle.ofBaseUnits(22, Degrees)));
    }
}