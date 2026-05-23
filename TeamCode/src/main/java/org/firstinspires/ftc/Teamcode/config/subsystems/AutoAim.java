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
    public static final double TICKS_PER_DEGREE = 5.37; // Adjust to your specific motor/gearing

    // --- PID COEFFICIENTS (Tune these in order: P, then D, then I) ---
    public static double kP = 0.035;
    public static double kI = 0.002;
    public static double kD = 0.0005;

    // --- PID MEMORY VARIABLES ---
    private double lastError = 0;
    private double integralSum = 0;
    private double maxIntegralWindup = 25.0; // Caps the max error accumulation to prevent crazy overshoots

    public AutoAim(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        timer.reset();
    }

    public void updateAim(double robotX, double robotY, double robotHeadingDeg) {
        // 1. Calculate time delta (dt) since last loop iteration
        double dt = timer.seconds();
        timer.reset();

        // Protect against zero division or extreme lag spikes on initialization
        if (dt <= 0 || dt > 0.2) dt = 0.02;

        // 2. Track absolute turret angle relative to the field floor
        double localTurretDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
        double currentTurretFieldDeg = robotHeadingDeg + localTurretDeg;

        // 3. Compute vector angle to goal
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double targetFieldAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // 4. Calculate error and normalize it to the shortest path (-180 to 180)
        double error = targetFieldAngleDeg - currentTurretFieldDeg;
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;

        // --- PID MATH ENGINE ---

        // Proportional term: Power scales linearly with current error magnitude
        double pTerm = kP * error;

        // Integral term: Accumulates error over time to overcome mechanical resistance
        integralSum += error * dt;
        // Cap the sum (anti-windup mechanism) so it doesn't build up massive uncontrollable energy
        integralSum = Math.max(-maxIntegralWindup, Math.min(maxIntegralWindup, integralSum));
        double iTerm = kI * integralSum;

        // Derivative term: Measures the rate of change of the error to act as a brake
        double derivative = (error - lastError) / dt;
        double dTerm = kD * derivative;

        // Save current error for the next loop calculation
        lastError = error;

        // Total raw output power required
        double totalPower = pTerm + iTerm + dTerm;

        // 5. Deadzone & Safe Power Ceiling Constraint
        if (Math.abs(error) < 0.7) {
            turretMotor.setPower(0);
            integralSum = 0; // Clear accumulated history when on target
            return;
        }

        // Limit the absolute maximum power delivered to the turret motor structure
        double finalPower = Math.max(-0.6, Math.min(0.6, totalPower));
        turretMotor.setPower(finalPower);
    }

    // Clear out PID memory variables when manually re-centering or restarting tracking states
    public void resetPIDMemory() {
        lastError = 0;
        integralSum = 0;
        timer.reset();
    }
}