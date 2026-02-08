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

public class IntakeSubsystem implements Subsystem {

    private final SparkFlex upperIntakeMotor;


    private final RelativeEncoder upperIntakeMotorEncoder;

    public IntakeSubsystem() {
        upperIntakeMotor = new SparkFlex(Ports.UPPER_INTAKE_PORT, MotorType.kBrushless);
        upperIntakeMotorEncoder = upperIntakeMotor.getEncoder();
    }

    public void startLoadFuel() {
        upperIntakeMotor.set(0.5);
    }

    public void stopLoadFuel() {
        upperIntakeMotor.set(0);
    }

    @Override
    public void periodic() {
        // Check to see if the limit switch has been triggered
        
    }
}
