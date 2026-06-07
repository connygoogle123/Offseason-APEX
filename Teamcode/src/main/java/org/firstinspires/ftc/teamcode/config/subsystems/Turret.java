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

    public static final double FIELD_START_OFFSET = 45.0;
    public final com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;

    private final ElapsedTime aimTimer = new ElapsedTime();
    private boolean autoAimEnabled = false;

    // hardware constants
    public static final double TURRET_TICKS_PER_DEGREE = 3.86; // good and accurate
    public static final double MAX_TURRET_DEG = 135.0;
    public static final double MIN_TURRET_DEG = -135.0;

    public static final boolean INVERT_CHASSIS_TRACKING = false;

    // --- TUNED SMOOTH PID PARAMETERS ---
    public double turret_kP = 0.012;
    public double turret_kI = 0.000;
    public double turret_kD = 0.004; // Gentle shock absorber to catch overshoot
    public double turret_kF = 0.000; // Left at zero to prevent runaways

    private double lastTurretError = 0;
    private double turretIntegralSum = 0;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "pinpoint");

        configurePinpointHardware();

        // Standardized direction control
        turret.setDirection(DcMotorEx.Direction.FORWARD);

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

        // Pull active heading directly from the Pinpoint gyro chip
        double robotHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

        // Target angle execution
        double targetLocalDeg;
        if (INVERT_CHASSIS_TRACKING) {
            targetLocalDeg = robotHeadingDeg + FIELD_START_OFFSET;
        } else {
            targetLocalDeg = -robotHeadingDeg + FIELD_START_OFFSET;
        }

        // Bound targets to physical safety travel limits
        targetLocalDeg = Math.max(MIN_TURRET_DEG, Math.min(MAX_TURRET_DEG, targetLocalDeg));

        // Read local physical encoder position
        double localTurretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;

        // --- FIXED 90-DEGREE STRAP WITH MULTIPLIER SCALING ---
        double error = (targetLocalDeg * 0.95) - localTurretDeg;

        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // PID calculations
        double pTerm = turret_kP * error;

        turretIntegralSum += error * dt;
        double iTerm = turret_kI * turretIntegralSum;

        double dTerm = turret_kD * (error - lastTurretError) / dt;
        lastTurretError = error;

        double totalPower = pTerm + iTerm + dTerm;

        // --- DYNAMIC DEADBAND TO CATCH LARGE TURN MOMENTUM ---
        double dynamicDeadband = (Math.abs(targetLocalDeg) > 60) ? 4.0 : 2.0;
        if (Math.abs(error) < dynamicDeadband) {
            turret.setPower(0);
            return;
        }

        // --- POWER LIMIT COMFORTABLY SET TO 30% ---
        double finalPower = Math.max(-0.30, Math.min(0.30, totalPower));
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