package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Turret {
    // components
    public final DcMotorEx turret;
    public final com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;

    private final ElapsedTime aimTimer = new ElapsedTime();
    private boolean autoAimEnabled = false;

    // hardware constants
    public static final double TURRET_TICKS_PER_DEGREE = 3.736;
    public static final double MAX_TURRET_DEG = 135.0;
    public static final double MIN_TURRET_DEG = -135.0;

    // --- ORIENTATION CONFIGURATION ---
    public static final boolean INVERT_CHASSIS_TRACKING = false;
    public static final boolean INVERT_TURRET_MOTOR_DIRECTION = false;

    // --- TUNED SMOOTH PID PARAMETERS ---
    public double turret_kP = 0.015;
    public double turret_kI = 0.000;
    public double turret_kD = 0.007;
    public double turret_kF = 0.000;

    private double lastTurretError = 0;
    private double turretIntegralSum = 0;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "pinpoint");

        configurePinpointHardware();

        if (INVERT_TURRET_MOTOR_DIRECTION) {
            turret.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            turret.setDirection(DcMotorEx.Direction.FORWARD);
        }


        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.autoAimEnabled = false;
        turret.setPower(0);
        aimTimer.reset();
    }

    private void configurePinpointHardware() {
        pinpoint.setOffsets(-100.0, 60.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD,
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        pinpoint.recalibrateIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void setAutoAimEnabled(boolean enabled) {
        this.autoAimEnabled = enabled;
        if (!enabled) {
            turret.setPower(0);
            resetAimPIDMemory();
        }
    }

    public boolean isAutoAimEnabled() { return autoAimEnabled; }

    private void runAutoAimPID() {
        double dt = aimTimer.seconds();
        aimTimer.reset();
        if (dt <= 0 || dt > 0.2) dt = 0.02;


        double robotHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

        double targetLocalDeg;
        if (INVERT_CHASSIS_TRACKING) {
            targetLocalDeg = robotHeadingDeg;
        } else {
            targetLocalDeg = -robotHeadingDeg;
        }

        while (targetLocalDeg > 180) targetLocalDeg -= 360;
        while (targetLocalDeg < -180) targetLocalDeg += 360;

        targetLocalDeg = Math.max(MIN_TURRET_DEG, Math.min(MAX_TURRET_DEG, targetLocalDeg));

        double localTurretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;

        double error = targetLocalDeg - localTurretDeg;

        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // PID
        double pTerm = turret_kP * error;

        turretIntegralSum += error * dt;
        double iTerm = turret_kI * turretIntegralSum;

        double dTerm = turret_kD * (error - lastTurretError) / dt;
        lastTurretError = error;

        double totalPower = pTerm + iTerm + dTerm;

        if (Math.abs(error) < 1.5) {
            turret.setPower(0);
            return;
        }

// assymetric clamping
        double finalPower = totalPower;
        if (finalPower < 0) {

            finalPower = Math.max(-0.50, finalPower);
        } else {

            finalPower = Math.min(0.4, finalPower);
        }

        // added different powers for turret direction to even out gear mesh
        if (Math.abs(error) < 5.0 && Math.abs(dTerm) > 0.02) {
            if (error > 0) {

                finalPower *= 0.60;
            } else {
                // standard dampening (25% power cut)
                finalPower *= 0.75;
            }
        }

        // Slew rate
        double currentPower = turret.getPower();
        double maxPowerChange = 0.06;
        if (finalPower - currentPower > maxPowerChange) {
            finalPower = currentPower + maxPowerChange;
        } else if (currentPower - finalPower > maxPowerChange) {
            finalPower = currentPower - maxPowerChange;
        }

        turret.setPower(finalPower);
    }

    private void resetAimPIDMemory() {
        lastTurretError = 0;
        turretIntegralSum = 0;
    }

    public void update() {
        pinpoint.update();
        if (autoAimEnabled) {
            runAutoAimPID();
        }
    }
}