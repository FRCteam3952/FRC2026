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
    // TODO pivot

    private final SparkFlex intakeMotor;
    private final RelativeEncoder intakeMotorEncoder;

    public IntakeSubsystem() {
        intakeMotor = new SparkFlex(Ports.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        intakeMotorEncoder = intakeMotor.getEncoder();
    }

    public void startLoadFuel() {
        intakeMotor.set(0.4);
    }

    public void stopLoadFuel() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        // Check to see if the limit switch has been triggered
        
    }
}
