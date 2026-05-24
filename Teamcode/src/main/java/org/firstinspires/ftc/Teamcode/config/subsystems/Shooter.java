package org.firstinspires.ftc.Teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Teamcode.config.subsystems.RGB;

public class Shooter {
    public enum ShooterState {
        IDLE, SPINNING_UP, READY, FEEDING
    }
    private ShooterState state = ShooterState.IDLE;
    private final DcMotorEx flywheelMotorLeft;
    private final DcMotorEx flywheelMotorRight;
    private final ElapsedTime feedTimer = new ElapsedTime();

    // tune everything
    private double targetVelocity = 0;
    private double velocityTolerance = 50;

    private double gateClosed = 1.0;
    private double gateOpen = 0.7;
    private double feedTime = 0.25;

    private double P = 0;
    private double F = 0;

    public Shooter(HardwareMap hardwareMap) {
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");

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
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }
    public void requestSpinUp(double velocity) {
        targetVelocity = velocity;
        state = ShooterState.SPINNING_UP;
    }
    public void requestStop() {
        state = ShooterState.IDLE;
    }
    public void requestFeed() {
        if (state == ShooterState.READY) {
            feedTimer.reset();
            state = ShooterState.FEEDING;
        }
    }
    public void update() {
        switch (state) {
            case IDLE:
                flywheelMotorLeft.setPower(0);
                flywheelMotorRight.setPower(0);
                break;

            case SPINNING_UP:
                flywheelMotorLeft.setVelocity(targetVelocity);
                flywheelMotorRight.setVelocity(targetVelocity);

                if (atSpeed()) {
                    state = ShooterState.READY;
                }
                break;

            case READY:
                flywheelMotorLeft.setVelocity(targetVelocity);
                flywheelMotorRight.setVelocity(targetVelocity);

                // if speed drops, go back to spinning up
                if (!atSpeed()) {
                    state = ShooterState.SPINNING_UP;
                }
                break;

            case FEEDING:
                flywheelMotorLeft.setVelocity(targetVelocity);
                flywheelMotorRight.setVelocity(targetVelocity);

                if (feedTimer.seconds() >= feedTime) {

                    if (atSpeed()) {
                        state = ShooterState.READY;
                    } else {
                        state = ShooterState.SPINNING_UP;
                    }
                }
                break;
        }
    }

    public boolean atSpeed() {
        return Math.abs(targetVelocity - getAverageVelocity()) <= velocityTolerance;
    }

    public double getLeftVelocity() {
        return flywheelMotorLeft.getVelocity();
    }

    public double getRightVelocity() {
        return flywheelMotorRight.getVelocity();
    }

    public double getAverageVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public ShooterState getState() {
        return state;
    }

}