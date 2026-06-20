package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Shooter {
    public enum ShooterState {
        IDLE, RUNNING
    }
    private ShooterState state = ShooterState.IDLE;
    private final DcMotorEx flywheelMotorLeft;
    private final DcMotorEx flywheelMotorRight;
    private final Servo hoodServo;
    public final Servo gate;

    private double targetVelocity = 0;

    public final double gateClosed = 1.0;
    public final double gateOpen = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);
        gate = hardwareMap.get(Servo.class, "gate");

        flywheelMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorConfigurationType leftType = flywheelMotorLeft.getMotorType().clone();
        leftType.setAchieveableMaxRPMFraction(1.0);
        flywheelMotorLeft.setMotorType(leftType);

        MotorConfigurationType rightType = flywheelMotorRight.getMotorType().clone();
        rightType.setAchieveableMaxRPMFraction(1.0);
        flywheelMotorRight.setMotorType(rightType);

        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(40.0, 0, 0, 14.0);
        flywheelMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheelMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        gate.setPosition(gateClosed);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setManualHoodPosition(double scaledPosition) {
        scaledPosition = Math.max(0.0, Math.min(1.0, scaledPosition));
        hoodServo.setPosition(scaledPosition);
    }

    public void requestSpinUp(double velocity) {
        targetVelocity = velocity;
        state = ShooterState.RUNNING;
    }

    public void requestStop() {
        state = ShooterState.IDLE;
    }

    public void toggleGate() {
        if (Math.abs(gate.getPosition() - gateClosed) < 0.1) {
            openGate();
        } else {
            closeGate();
        }
    }

    public void openGate() {
        gate.setPosition(gateOpen);
    }

    public void closeGate() {
        gate.setPosition(gateClosed);
    }

    public void update() {
        if (state == ShooterState.RUNNING) {
            flywheelMotorLeft.setVelocity(targetVelocity);
            flywheelMotorRight.setVelocity(targetVelocity);
        } else {
            flywheelMotorLeft.setPower(0);
            flywheelMotorRight.setPower(0);
        }
    }

    public double getAverageVelocity() {
        return (Math.abs(flywheelMotorLeft.getVelocity()) + Math.abs(flywheelMotorRight.getVelocity())) / 2.0;
    }

    public double getTargetVelocity() { return targetVelocity; }
    public ShooterState getState() { return state; }
    public double getHoodPosition() { return hoodServo.getPosition(); }

    public void aimForDistance(double distance) {
        double velocity = org.firstinspires.ftc.teamcode.config.tests.shootertest.getSpeed(distance);

        velocity *= 1.045;

        double hoodDegrees = org.firstinspires.ftc.teamcode.config.tests.shootertest.getAngle(distance);

        setTargetVelocity(velocity);

        double minDegrees = 25.0;
        double maxDegrees = 60.0;
        double scaledPos = (hoodDegrees - minDegrees) / (maxDegrees - minDegrees);

        setManualHoodPosition(scaledPos);
    }
}