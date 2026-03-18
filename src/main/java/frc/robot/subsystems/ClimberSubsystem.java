package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;


//NOTE: this is a clean ClimberSubsystem, with no variables/methods. To be added when climber is added
public class ClimberSubsystem extends SubsystemBase {
    // Instance variables go here
    TalonFX elevatorMotor = new TalonFX(Constants.Ports.CLIMBER_CAN_ID);
    double prevMotorPosition = 0; // what's the starting position
    int rotationCount = 0;

    // Methods go here
    public ClimberSubsystem() {
        elevatorMotor.setPosition(Angle.ofBaseUnits(0, Degrees));
    }

    public double getRawPositionDegrees() {
        return elevatorMotor.getPosition().getValue().in(Degrees);
    }

    public double getTotalPositionDegrees() {
        return getRawPositionDegrees() + 360 * rotationCount;
    }

    @Override
    public void periodic() {
        double newMotorPosition = getRawPositionDegrees();
        if (newMotorPosition > 300 && prevMotorPosition < 60) {
            // it just wrapped around down
            rotationCount -= 1; 
        } else if (newMotorPosition < 60 && prevMotorPosition > 300) {
            // wrapped up 
            rotationCount += 1;
        }
        prevMotorPosition = newMotorPosition;

        System.out.println("getRawPositionDegrees() = " + getRawPositionDegrees());
        System.out.println("getTotalPositionDegrees() = " + getTotalPositionDegrees());
    }
}
