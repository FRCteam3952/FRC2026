package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
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

public class IntakeSubsystem extends SubsystemBase {
    // TODO pivot
    // private final TalonFX leftPivot;
    // private final TalonFX rightPivot;

    // private final SparkMax intakeMotor;
    // private final RelativeEncoder intakeMotorEncoder;

    public IntakeSubsystem() {
        // intakeMotor = new SparkMax(Ports.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        // intakeMotorEncoder = intakeMotor.getEncoder();

        // leftPivot = new TalonFX(Ports.LEFT_INTAKE_PIVOT_CAN_ID);
        // rightPivot = new TalonFX(Ports.RIGHT_INTAKE_PIVOT_CAN_ID);
    }

    public void setPivotSpeeds(double speed) {
        // leftPivot.set(speed);
        // rightPivot.set(speed);
    }

    public void startPivotUpwards() {
        setPivotSpeeds(0.2);
    }

    public void stopPivot() {
        setPivotSpeeds(0.0);
    }

    public void startPivotDown() {
        setPivotSpeeds(-0.2);
    }

    public void startLoadFuel() {
        // intakeMotor.set(0.45);
    }

    public void stopLoadFuel() {
        // intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        
    }
}
