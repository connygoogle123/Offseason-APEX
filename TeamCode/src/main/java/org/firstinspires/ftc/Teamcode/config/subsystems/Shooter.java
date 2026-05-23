package org.firstinspires.ftc.Teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    public enum ShooterState { IDLE, SPINNING_UP, READY, FEEDING }
    
    private ShooterState state = ShooterState.IDLE;
    private final DcMotorEx flywheelLeft, flywheelRight;
    private final Servo gate;
    private final RGB stateLight;
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final HardwareMap hwMap;

    // --- TUNEABLE CONSTANTS ---
    public static double GATE_CLOSED = 1.0;
    public static double GATE_OPEN = 0.7;
    public static double FEED_TIME_SEC = 0.25;
    public static double VELOCITY_TOLERANCE = 50.0;
    
    private double targetVelocity = 0;

    public Shooter(HardwareMap hardwareMap) {
        this.hwMap = hardwareMap;
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        gate = hardwareMap.get(Servo.class, "gate");
        stateLight = new RGB(hardwareMap.get(Servo.class, "rgb2"));

        // Configure Motors
        configureMotor(flywheelLeft, DcMotorSimple.Direction.REVERSE);
        configureMotor(flywheelRight, DcMotorSimple.Direction.FORWARD);
        
        gate.setPosition(GATE_CLOSED);
    }

    private void configureMotor(DcMotorEx motor, DcMotorSimple.Direction dir) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(dir);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Allows wheels to coast down
        
        // Disable internal limits to allow PIDF to work at max capacity
        MotorConfigurationType type = motor.getMotorType().clone();
        type.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(type);
    }

    public void update() {
        double currentVel = getAverageVelocity();

        switch (state) {
            case IDLE:
                stopMotors();
                gate.setPosition(GATE_CLOSED);
                stateLight.blue();
                break;

            case SPINNING_UP:
                runMotors(targetVelocity);
                gate.setPosition(GATE_CLOSED);
                stateLight.azure();
                if (isAtTarget(currentVel)) state = ShooterState.READY;
                break;

            case READY:
                runMotors(targetVelocity);
                stateLight.green();
                if (!isAtTarget(currentVel)) state = ShooterState.SPINNING_UP;
                break;

            case FEEDING:
                runMotors(targetVelocity);
                gate.setPosition(GATE_OPEN);
                stateLight.orange();
                
                if (feedTimer.seconds() >= FEED_TIME_SEC) {
                    gate.setPosition(GATE_CLOSED);
                    state = isAtTarget(currentVel) ? ShooterState.READY : ShooterState.SPINNING_UP;
                }
                break;
        }
    }

    // --- HELPER METHODS ---

    private void runMotors(double vel) {
        flywheelLeft.setVelocity(vel);
        flywheelRight.setVelocity(vel);
    }

    private void stopMotors() {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    }

    private boolean isAtTarget(double currentVel) {
        return Math.abs(targetVelocity - currentVel) <= VELOCITY_TOLERANCE;
    }

    public double getAverageVelocity() {
        return (flywheelLeft.getVelocity() + flywheelRight.getVelocity()) / 2.0;
    }

    public void requestFeed() {
        // Only allow feeding if we are actually ready to shoot!
        if (state == ShooterState.READY) {
            feedTimer.reset();
            state = ShooterState.FEEDING;
        }
    }

    public void requestSpinUp(double velocity) {
        this.targetVelocity = velocity;
        this.state = ShooterState.SPINNING_UP;
    }
}
