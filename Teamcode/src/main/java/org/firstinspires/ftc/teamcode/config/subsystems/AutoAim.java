package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "AUTO    AIM TURRET (TOGGLE ONLY)", group = "TeleOp")
public class AutoAim extends LinearOpMode {

    private DcMotorEx turret;
    private com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;
    private final ElapsedTime timer = new ElapsedTime();

    // --- TOGGLE STATE MEMORY ---
    private boolean autoAimEnabled = false;
    private boolean lastRightBumperState = false;

    // --- HARDCODED TARGET GOAL ---
    public static final double GOAL_X = 0.0;
    public static final double GOAL_Y = 0.0;
    public static final double TICKS_PER_DEGREE = 5.37;

    // --- TURRET PID TUNING ---
    public static double kP = 0.035;
    public static double kI = 0.002;
    public static double kD = 0.0005;

    // --- 270 DEGREE TURRET LIMITS ---
    public static final double MAX_TURRET_DEG = 135.0;
    public static final double MIN_TURRET_DEG = -135.0;

    private double lastError = 0;
    private double integralSum = 0;
    private final double maxIntegralWindup = 25.0;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Toggle-Only Auto Aim Ready!");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            pinpoint.update();

            Pose2D robotPose = pinpoint.getPosition();
            double robotX = robotPose.getX(DistanceUnit.INCH);
            double robotY = robotPose.getY(DistanceUnit.INCH);
            double robotHeadingDeg = robotPose.getHeading(AngleUnit.DEGREES);

            // --- TOGGLE LOGIC ---
            // Checks for a "rising edge" (when you first press the bumper down)
            // so it only flips the state once per click.
            if (gamepad1.right_bumper && !lastRightBumperState) {
                autoAimEnabled = !autoAimEnabled; // Flip the switch (ON -> OFF or OFF -> ON)
                if (!autoAimEnabled) {
                    resetPIDMemory();
                }
            }
            lastRightBumperState = gamepad1.right_bumper; // Save button state for next loop

            // --- EXECUTION CODE ---
            if (autoAimEnabled) {
                runAutoAimPID(robotX, robotY, robotHeadingDeg);
                telemetry.addData("Turret Mode", "AUTO TRACKING [ON]");
            } else {
                turret.setPower(0); // Safely sit still when tracking is turned off
                telemetry.addData("Turret Mode", "DISABLED [OFF]");
            }

            telemetry.addData("Pinpoint X (Inches)", robotX);
            telemetry.addData("Pinpoint Y (Inches)", robotY);
            telemetry.addData("Pinpoint Heading (Deg)", robotHeadingDeg);
            telemetry.addData("Turret Position (Ticks)", turret.getCurrentPosition());
            telemetry.update();
        }
    }

    private void runAutoAimPID(double robotX, double robotY, double robotHeadingDeg) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.2) dt = 0.02;

        double localTurretDeg = turret.getCurrentPosition() / TICKS_PER_DEGREE;
        double currentTurretFieldDeg = robotHeadingDeg + localTurretDeg;

        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double targetFieldAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));

        double error = targetFieldAngleDeg - currentTurretFieldDeg;
        error = ((error + 180) % 360 + 360) % 360 - 180;

        double targetLocalDeg = targetFieldAngleDeg - robotHeadingDeg;
        targetLocalDeg = Math.max(MIN_TURRET_DEG, Math.min(MAX_TURRET_DEG, targetLocalDeg));
        error = targetLocalDeg - localTurretDeg;

        double pTerm = kP * error;
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralWindup, Math.min(maxIntegralWindup, integralSum));
        double iTerm = kI * integralSum;
        double dTerm = kD * (error - lastError) / dt;
        lastError = error;

        double totalPower = pTerm + iTerm + dTerm;

        if (Math.abs(error) < 0.7) {
            turret.setPower(0);
            integralSum = 0;
            return;
        }

        double finalPower = Math.max(-0.6, Math.min(0.6, totalPower));
        turret.setPower(finalPower);
    }

    private void resetPIDMemory() {
        lastError = 0;
        integralSum = 0;
    }
}