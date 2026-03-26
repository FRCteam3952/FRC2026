package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
    // TODO pivot
    // private final SparkMax leftPivot;
    private final SparkMax rightPivot;
    private final PIDController rightPivotPID = new PIDController(2, 0, 0);
    // private final PIDController leftPivotPID = new PIDController(0.5, 0, 0);

    private final SparkMax intakeMotor;

    private final AbsoluteEncoder rightAbsoluteEncoder;
    // private final RelativeEncoder leftEncoder;

    private final double downPivotPos = 0.2530;
    private final double jigglePivotPos = 0.4000;
    private final double middlePivotPos = 0.5640;
    private final double upPivotPos = 0.7680;

    // private final double leftEncoderRange = 7.5;
    // private final double rightEncoderRange = upPivotPos - downPivotPos;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(Ports.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

        // leftPivot = new SparkMax(Ports.LEFT_INTAKE_PIVOT_CAN_ID, MotorType.kBrushless);
        rightPivot = new SparkMax(Ports.RIGHT_INTAKE_PIVOT_CAN_ID, MotorType.kBrushless);
        rightAbsoluteEncoder = rightPivot.getAbsoluteEncoder();
        // leftEncoder = leftPivot.getEncoder();

        rightPivotPID.setIntegratorRange(0, 0.10); // not being used
        // leftPivotPID.setIntegratorRange(0, 0.10);
        rightPivotPID.setSetpoint(upPivotPos);
        // leftPivotPID.setSetpoint(upPivotPos);
        // zeroLeftEncoder();
    }

    // public void zeroLeftEncoder() {
    //     double rightPosition = rightAbsoluteEncoder.getPosition();
    //     double leftPosition = (rightPosition - downPivotPos) / rightEncoderRange * leftEncoderRange;
    //     leftEncoder.setPosition(leftPosition);
    // }

    // public double getNormalizedLeftPosition() {
    //     return leftEncoder.getPosition() / leftEncoderRange * rightEncoderRange + downPivotPos;
    // }

    public Command getRunIntakeCommand() {
        Command noninteruptible = Commands.run(this::startLoadFuel, this);
        return noninteruptible;
    }

    public Command getLowerIntakeCommand() {
        Command noninteruptible = Commands.run(()-> {
            this.setSetpoint(downPivotPos);
        }, this);        
        return noninteruptible;
    }

    public double getNormalizedRightPosition() {
        return rightAbsoluteEncoder.getPosition();
    }

    public void pivotTowardPositive() {
        this.rightPivot.set(0.2);
    }
    
    public void pivotTowardNegative() {
        this.rightPivot.set(-0.2);
    }
    
    public void pivotStop() {
        this.rightPivot.set(-0.0);
    }

    public void setPivotSpeeds(double speed) {
        System.out.println("speed = " + speed);
        if (speed > 0.05) {
            rightPivot.set(speed + 0.1);
        } else {
            rightPivot.set(speed);
        }
    }

    public void startLoadFuel() {
        intakeMotor.set(0.35);
    }

    public void stopLoadFuel() {
        intakeMotor.set(0);
    }

    public void toggleIntake() {
        if (intakeMotor.get() == 0) {
            startLoadFuel();
        } else {
            stopLoadFuel();
        }
    }

    public void runPivotPID() {
        setPivotSpeeds(-rightPivotPID.calculate(getNormalizedRightPosition()));
        // setPivotSpeeds(leftPivotPID.calculate(getNormalizedRightPosition()));
    }

    private void setSetpoint(double setpoint) {
        rightPivotPID.setSetpoint(setpoint);
        // leftPivotPID.setSetpoint(setpoint);
    }

    public void togglePivot() {
        if (rightPivotPID.getSetpoint() == downPivotPos) {
            setPivotUp();
        } else if (rightPivotPID.getSetpoint() == middlePivotPos) {
            setPivotDown();
        } else if (rightPivotPID.getSetpoint() == upPivotPos) {
            setPivotDown();
        } else {
            // this branch should never execute
            setPivotDown();
        }
    }

    public void setPivotDown() {
        setSetpoint(downPivotPos);
    }

    public void setPivotUp() {
        setSetpoint(upPivotPos);
    }

    public void setPivotMiddle() {
        setSetpoint(middlePivotPos);
    }

    public void setPivotJiggle() {
        setSetpoint(jigglePivotPos);
    }

    @Override
    public void periodic() {
        runPivotPID();
        
        System.out.println("right =" + getNormalizedRightPosition());
        //System.out.println("left = " + getNormalizedLeftPosition());
    }
}
