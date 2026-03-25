package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Note: DISABLED in flags right now.

public class ClimberSubsystem extends SubsystemBase {
    // Instance variables go here
    TalonFX elevatorMotor = new TalonFX(Constants.Ports.CLIMBER_CAN_ID);
    double prevMotorPosition = 0; // what's the starting position
    int rotationCount = 0;

    // Methods go here
    public ClimberSubsystem() {
        elevatorMotor.setPosition(Angle.ofBaseUnits(0, Degrees));
    }

    public void startGoingUp() {
        elevatorMotor.set(0.4);
    }

    public void stop() {
        elevatorMotor.set(0);
    }

    public void startGoingDown() {
        elevatorMotor.set(-0.4);
    }

    // public boolean isAtTop() {
     
    // }

    public boolean isAtBottom() {
        return getTotalPositionDegrees() < 30;
    }

    public double getRawPositionDegrees() {
        return elevatorMotor.getPosition().getValue().in(Degrees);
    }

    public double getTotalPositionDegrees() {
        return getRawPositionDegrees() + 360 * rotationCount;
    }

    // public void toggleHopper() {
    //
    // }

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
