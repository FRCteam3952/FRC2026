package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
    // TODO pivot
    // private final SparkMax leftPivot;
    private final TalonFX pivotMotor;
    private final PIDController rightPivotPID = new PIDController(1.25, 0, 0);
    // private final PIDController leftPivotPID = new PIDController(0.5, 0, 0);

    private final SparkMax intakeMotor;

    private final AbsoluteEncoder pivotAbsoluteEncoder;
    // private final RelativeEncoder leftEncoder;

    private final double downPivotPos = 0.1300; // is: 0.1170
    // private final double jigglePivotPos = 0.3000;  // tune me?
    // private final double middlePivotPos = 0.6000; // tune me?
    private final double upPivotPos = 0.580; // 0.6000 max
    private final double pivotRange = upPivotPos - downPivotPos;

    public boolean timWantsTheIntakeToMove = false;

    // private final double leftEncoderRange = 7.5;
    // private final double rightEncoderRange = upPivotPos - downPivotPos;

    public void setTimWantsTheIntakeToMove(boolean value) {
        timWantsTheIntakeToMove = value;
    }

    public IntakeSubsystem(AbsoluteEncoder intakePivotAbsoluteEncoder) {
        intakeMotor = new SparkMax(Ports.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

        // leftPivot = new SparkMax(Ports.LEFT_INTAKE_PIVOT_CAN_ID, MotorType.kBrushless);
        pivotMotor = new TalonFX(Ports.RIGHT_INTAKE_PIVOT_CAN_ID);
        pivotAbsoluteEncoder = intakePivotAbsoluteEncoder;
        // leftEncoder = leftPivot.getEncoder();

        // rightPivotPID.setIntegratorRange(0, 0.10); // not being used
        // leftPivotPID.setIntegratorRange(0, 0.10);
        rightPivotPID.setSetpoint(upPivotPos);
        // leftPivotPID.setSetpoint(upPivotPos);
        // zeroLeftEncoder();
    }

    public double getNormalizedRightPosition() {
        return pivotAbsoluteEncoder.getPosition();
    }

    public void pivotTowardPositive() {
        this.pivotMotor.set(0.2);
    }
    
    public void pivotTowardNegative() {
        this.pivotMotor.set(-0.2);
    }
    
    public void pivotStop() {
        this.pivotMotor.set(-0.0);
    }

    public void setPivotSpeeds(double speed) {
        if (speed > 0.05) {
            // might be same as kP increase
            double position = getNormalizedRightPosition();
            double intakeAngleFromHorizontal = (position - downPivotPos) / pivotRange * (Math.PI / 2);
            double correctionConstant = Math.cos(intakeAngleFromHorizontal);
            double feedforward = correctionConstant * 0.1;
            // feedforward = 0.1; // testing
            pivotMotor.set(speed + feedforward);
        } else {
            pivotMotor.set(speed);
        }
    }

    public void startLoadFuel() {
        intakeMotor.set(0.7);
    }
    
    public void startSpitFuel() {
        intakeMotor.set(-0.7);
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
            setPivotSpeeds(-rightPivotPID.calculate(getNormalizedRightPosition()));
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
        
        //System.out.println("right =" + getNormalizedRightPosition());
        //System.out.println("left = " + getNormalizedLeftPosition());
    }
}
