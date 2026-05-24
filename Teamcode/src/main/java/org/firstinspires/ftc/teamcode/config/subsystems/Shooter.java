package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.subsystems.RGB;

public class Shooter {
    public enum ShooterState {
        IDLE, SPINNING_UP, READY, FEEDING
    }
    private ShooterState state = ShooterState.IDLE;
    private final DcMotorEx flywheelMotorLeft;
    private final DcMotorEx flywheelMotorRight;
    public final Servo gate;
    private final RGB stateLight;
    private final ElapsedTime feedTimer = new ElapsedTime();

    // --- AUTO AIM HARDWARE & CONFIGURATION ---
    private final DcMotorEx turret;
    private final com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;
    private final ElapsedTime aimTimer = new ElapsedTime();
    private boolean autoAimEnabled = false;

    // --- FIELD GOAL COORDINATES (0,0 tracking to face starting wall position) ---
    public static final double GOAL_X = 0.0;
    public static final double GOAL_Y = 0.0;
    public static final double TURRET_TICKS_PER_DEGREE = 5.37;

    // --- TURRET PID TUNING ---
    public static double turret_kP = 0.035;
    public static double turret_kI = 0.002;
    public static double turret_kD = 0.0005;

    // --- 270 DEGREE TURRET PHYSICAL LIMITS ---
    public static final double MAX_TURRET_DEG = 135.0;
    public static final double MIN_TURRET_DEG = -135.0;

    private double lastTurretError = 0;
    private double turretIntegralSum = 0;
    private final double maxIntegralWindup = 25.0;

    // --- SHOOTER CONFIGURATION ---
    private double targetVelocity = 0;
    private double velocityTolerance = 50;

    public double gateClosed = 0.0;
    public double gateOpen = 0.5;
    private double feedTime = 1.0;

    private double P = 0;
    private double F = 0;

    public Shooter(HardwareMap hardwareMap) {
        // Initialize Flywheels and Gate
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        gate = hardwareMap.get(Servo.class, "gate");
        Servo rgbServo = hardwareMap.get(Servo.class, "rgb2");

        // Initialize Auto Aim Hardware
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "pinpoint");

        // Reset and setup turret motor
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset and setup flywheel encoders
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

        setPIDF(33.2, 13.1);
        gate.setPosition(gateClosed);
        stateLight = new RGB(rgbServo);

        aimTimer.reset();
    }

    public void setPIDF(double p, double f) {
        P = p;
        F = f;

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheelMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    // --- AUTO AIM CONTROL METHODS ---
    public void setAutoAimEnabled(boolean enabled) {
        this.autoAimEnabled = enabled;
        if (!enabled) {
            turret.setPower(0);
            resetAimPIDMemory();
        }
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    private void runAutoAimPID() {
        double dt = aimTimer.seconds();
        aimTimer.reset();
        if (dt <= 0 || dt > 0.2) dt = 0.02;

        // Pull coordinates from Pinpoint
        pinpoint.update();
        Pose2D robotPose = pinpoint.getPosition();
        double robotX = robotPose.getX(DistanceUnit.INCH);
        double robotY = robotPose.getY(DistanceUnit.INCH);
        double robotHeadingDeg = robotPose.getHeading(AngleUnit.DEGREES);

        // Get current relative turret angle
        double localTurretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        double currentTurretFieldDeg = robotHeadingDeg + localTurretDeg;

        // Trigonometry calculation to target coordinate
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double targetFieldAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Heading error boundary wrap (-180 to 180)
        double error = targetFieldAngleDeg - currentTurretFieldDeg;
        error = ((error + 180) % 360 + 360) % 360 - 180;

        // Protect physical wires by constraining target angles
        double targetLocalDeg = targetFieldAngleDeg - robotHeadingDeg;
        targetLocalDeg = Math.max(MIN_TURRET_DEG, Math.min(MAX_TURRET_DEG, targetLocalDeg));
        error = targetLocalDeg - localTurretDeg;

        // PID Math
        double pTerm = turret_kP * error;
        turretIntegralSum += error * dt;
        turretIntegralSum = Math.max(-maxIntegralWindup, Math.min(maxIntegralWindup, turretIntegralSum));
        double iTerm = turret_kI * turretIntegralSum;
        double dTerm = turret_kD * (error - lastTurretError) / dt;
        lastTurretError = error;

        double totalPower = pTerm + iTerm + dTerm;

        // Small deadband to prevent motor jitter when aligned
        if (Math.abs(error) < 0.7) {
            turret.setPower(0);
            turretIntegralSum = 0;
            return;
        }

        double finalPower = Math.max(-0.6, Math.min(0.6, totalPower));
        turret.setPower(finalPower);
    }

    private void resetAimPIDMemory() {
        lastTurretError = 0;
        turretIntegralSum = 0;
    }

    // --- MAIN SUBSYSTEM UPDATE METHOD ---
    public void update() {
        // Execute tracking loops if turned on
        if (autoAimEnabled) {
            runAutoAimPID();
        }

        // Flywheel and Gate State Machine
        switch (state) {
            case IDLE:
                flywheelMotorLeft.setPower(0);
                flywheelMotorRight.setPower(0);
                gate.setPosition(gateClosed);
                stateLight.blue();
                break;

            case SPINNING_UP:
                flywheelMotorLeft.setVelocity(targetVelocity);
                flywheelMotorRight.setVelocity(targetVelocity);
                gate.setPosition(gateClosed);
                stateLight.azure();

                if (atSpeed()) {
                    state = ShooterState.READY;
                }
                break;

            case READY:
                flywheelMotorLeft.setVelocity(targetVelocity);
                flywheelMotorRight.setVelocity(targetVelocity);
                gate.setPosition(gateClosed);
                stateLight.green();

                if (!atSpeed()) {
                    state = ShooterState.SPINNING_UP;
                }
                break;

            case FEEDING:
                flywheelMotorLeft.setVelocity(targetVelocity);
                flywheelMotorRight.setVelocity(targetVelocity);
                gate.setPosition(gateOpen);
                stateLight.orange();

                if (feedTimer.seconds() >= feedTime) {
                    gate.setPosition(gateClosed);

                    if (atSpeed()) {
                        state = ShooterState.READY;
                    } else {
                        state = ShooterState.SPINNING_UP;
                    }
                }
                break;
        }
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
            gate.setPosition(gateOpen);
            feedTimer.reset();
            state = ShooterState.FEEDING;
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
        return (Math.abs(getLeftVelocity()) + Math.abs(getRightVelocity())) / 2.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public ShooterState getState() {
        return state;
    }
}