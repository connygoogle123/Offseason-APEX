package org.firstinspires.ftc.Teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoAim {

    private final DcMotorEx turretMotor;
    private final ElapsedTime timer = new ElapsedTime();

    // --- FIELD CONFIGURATION ---
    public static final double GOAL_X = -72.0;
    public static final double GOAL_Y = 36.0;
    public static final double TICKS_PER_DEGREE = 5.37;

    // --- TUNING ---
    public static double kP = 0.035;
    public static double kI = 0.002;
    public static double kD = 0.0005;

    // --- 270 DEGREE TURRET LIMITS ---
    public static final double MAX_TURRET_DEG = 135.0;   // Adjust based on your hardware
    public static final double MIN_TURRET_DEG = -135.0;  // Typically symmetric around center

    // PID memory
    private double lastError = 0;
    private double integralSum = 0;
    private final double maxIntegralWindup = 25.0;

    public AutoAim(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        timer.reset();
    }

    public void updateAim(double robotX, double robotY, double robotHeadingDeg) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.2) dt = 0.02;

        // Current turret position (relative to robot)
        double localTurretDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        // Current absolute turret angle on field
        double currentTurretFieldDeg = robotHeadingDeg + localTurretDeg;

        // Target angle to goal
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double targetFieldAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Calculate shortest error
        double error = targetFieldAngleDeg - currentTurretFieldDeg;
        error = ((error + 180) % 360 + 360) % 360 - 180;  // Better normalization

        // === 270° TURRET LIMIT HANDLING ===
        double targetLocalDeg = targetFieldAngleDeg - robotHeadingDeg;

        // Clamp target to turret's physical range
        targetLocalDeg = Math.max(MIN_TURRET_DEG, Math.min(MAX_TURRET_DEG, targetLocalDeg));

        // Recalculate error using clamped target
        error = targetLocalDeg - localTurretDeg;

        // --- PID Control ---
        double pTerm = kP * error;
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralWindup, Math.min(maxIntegralWindup, integralSum));
        double iTerm = kI * integralSum;
        double dTerm = kD * (error - lastError) / dt;
        lastError = error;

        double totalPower = pTerm + iTerm + dTerm;

        if (Math.abs(error) < 0.7) {
            turretMotor.setPower(0);
            integralSum = 0;
            return;
        }

        double finalPower = Math.max(-0.6, Math.min(0.6, totalPower));
        turretMotor.setPower(finalPower);
    }

    public void resetPIDMemory() {
        lastError = 0;
        integralSum = 0;
        timer.reset();
    }
}