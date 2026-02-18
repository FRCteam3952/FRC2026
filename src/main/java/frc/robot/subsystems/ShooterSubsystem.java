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
import frc.robot.Constants.Ports;
public class ShooterSubsystem {
    private final SparkMax lowerIntakeMotor;
    private final SparkFlex roller;
    private final SparkFlex agitator;
    private final SparkMax flywheel;
    private final SparkMax upperIntakeMotor;


    public ShooterSubsystem() {
        lowerIntakeMotor = new SparkMax(Ports.LOWER_INTAKE_CAN_ID, MotorType.kBrushless);
        roller = new SparkFlex(Ports.ROLLER_CAN_ID, MotorType.kBrushless);
        agitator = new SparkFlex(Ports.AGITATOR_CAN_ID, MotorType.kBrushless);
        flywheel = new SparkMax(Ports.FLYWHEEL_CAN_ID, MotorType.kBrushless);
        upperIntakeMotor = new SparkMax(Ports.UPPER_INTAKE_CAN_ID, MotorType.kBrushless);
    }

    public void startLoadFuelFlywheel() {
        flywheel.set(-0.75);
    }

    // public void stopLoadFuelFlywheel() {
    //     flywheel.set(0);
    // }

    public void startLoadFuel() {
        lowerIntakeMotor.set(-0.5);
        roller.set(0.5);
        agitator.set(-0.5);
        upperIntakeMotor.set(0.5);

    }

    public void stopLoadFuel() {
        lowerIntakeMotor.set(0);
        roller.set(0);
        agitator.set(0);
        flywheel.set(0);
        upperIntakeMotor.set(0);    
    }

    // @Override
    // public void periodic() {
    //     // Check to see if the limit switch has been triggered
        
    // }
}
