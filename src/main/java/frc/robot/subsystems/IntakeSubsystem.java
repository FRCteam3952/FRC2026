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
    private final PIDController rightPivotPID = new PIDController(3.0, 0, 0);
    // private final PIDController leftPivotPID = new PIDController(0.5, 0, 0);

    private final SparkMax intakeMotor;

    private final AbsoluteEncoder rightAbsoluteEncoder;
    // private final RelativeEncoder leftEncoder;

    private final double downPivotPos = 0.080; // is: 0.059
    // private final double jigglePivotPos = 0.3000;  // tune me?
    // private final double middlePivotPos = 0.6000; // tune me?
    private final double upPivotPos = 0.5780; // 0.6080 max

    public boolean timWantsTheIntakeToMove = true;

    // private final double leftEncoderRange = 7.5;
    // private final double rightEncoderRange = upPivotPos - downPivotPos;

    public void setTimWantsTheIntakeToMove(boolean value) {
        timWantsTheIntakeToMove = value;
    }

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

    // public Command getRunIntakeCommand() {
    //     Command noninterruptible = Commands.run(this::startLoadFuel, this);
    //     return noninterruptible;
    // }

    // public Command getStopIntakeCommand() {
    //     Command noninterruptible = Commands.run(this::stopLoadFuel, this);
    //     return noninterruptible;
    // }

    // public Command getLowerIntakeCommand() {
    //     Command noninterruptible = Commands.run(this::setPivotDown, this);        
    //     return noninterruptible;
    // }

    // public Command getRaiseIntakeCommand() {
    //     Command noninterruptible = Commands.run(this::setPivotUp, this);
    //     return noninterruptible;
    // }

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
        // System.out.println("speed = " + speed);
        if (speed > 0.05) {
            rightPivot.set(speed + 0.1);
        } else {
            rightPivot.set(speed);
        }
    }

    public void startLoadFuel() {
        intakeMotor.set(0.55);
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
        // setPivotSpeeds(rightPivotPID.calculate(getNormalizedRightPosition()));

        if (timWantsTheIntakeToMove) {
            setPivotSpeeds(rightPivotPID.calculate(getNormalizedRightPosition()));
        } else {
            setPivotSpeeds(0);
        }
    }

    private void setSetpoint(double setpoint) {
        rightPivotPID.setSetpoint(setpoint);
        // leftPivotPID.setSetpoint(setpoint);
    }

    public void togglePivot() {
        // if (rightPivotPID.getSetpoint() == downPivotPos) {
        //     setPivotUp();
        // // } else if (rightPivotPID.getSetpoint() == middlePivotPos) {
        // //     setPivotDown();
        // } else if (rightPivotPID.getSetp/oint() == upPivotPos) {
        //     setPivotDown();
        // } else {
            // this branch should never execute
            setPivotDown();
        // }
    }

    public void setPivotDown() {
        setSetpoint(downPivotPos);
    }

    public void setPivotUp() {
        setSetpoint(upPivotPos);
    }

    // public void setPivotMiddle() {
    //     setSetpoint(middlePivotPos);
    // }

    // public void setPivotJiggle() {
    //     setSetpoint(jigglePivotPos);
    // }

    @Override
    public void periodic() {
        runPivotPID();
        
        // System.out.println("right =" + getNormalizedRightPosition());
        //System.out.println("left = " + getNormalizedLeftPosition());
    }
}
