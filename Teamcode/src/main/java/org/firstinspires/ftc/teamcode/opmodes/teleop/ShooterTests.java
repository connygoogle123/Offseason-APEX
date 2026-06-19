package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.tests.shootertest;

@TeleOp(name = "Shooter LUT Tester: Tuning Mode", group = "Testing")
public class ShooterTests extends LinearOpMode {

    private Shooter shooter;
    private double testDistance = 70.0;
    private double velocityOffset = 0.0;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Shooter Subsystem...");
        telemetry.update();

        shooter = new Shooter(hardwareMap);

        telemetry.addLine("Ready! Controls:");
        telemetry.addLine("D-Pad Left/Right -> Change Distance");
        telemetry.addLine("D-Pad Up/Down    -> Adjust Speed Offset (+/- 50 RPM)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. CHANGE SIMULATED DISTANCE (D-pad Left / Right)
            if (gamepad1.dpad_right && !dpadRightPressed) {
                testDistance += 5.0;
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

            if (gamepad1.dpad_left && !dpadLeftPressed) {
                testDistance = Math.max(0, testDistance - 5.0);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            // 2. ADJUST VELOCITY OFFSET ON THE FLY (D-pad Up / Down)
            if (gamepad1.dpad_up && !dpadUpPressed) {
                velocityOffset += 50.0;
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            if (gamepad1.dpad_down && !dpadDownPressed) {
                velocityOffset -= 50.0;
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            // 3. FETCH COORDS FROM INTERPLUT AUTOMATION LAYER
            double baseSpeed = shootertest.getSpeed(testDistance);
            double targetAngle = shootertest.getAngle(testDistance);

            double finalTargetSpeed = baseSpeed + velocityOffset;

            // 4. MOTOR AND SERVO ACTION
            if (gamepad1.a) {
                // Command the Flywheel Speed
                shooter.setTargetVelocity(finalTargetSpeed);
                shooter.requestSpinUp(finalTargetSpeed);

                // FIXED: Explicitly command the physical hood servo position!
                // Make sure your ShooterCalculations contains decimal values between 0.0 and 1.0!
                shooter.setManualHoodPosition(targetAngle);
            }
            else if (gamepad1.b) {
                shooter.requestStop();
                velocityOffset = 0;
            }

            // Fire the gate execution loop
            if (gamepad1.right_bumper) {
                shooter.requestFeed();
            }

            shooter.update();

            // 5. TUNING DIAGNOSTICS
            telemetry.addLine("=== LUT VELOCITY & HOOD TUNER ===");
            telemetry.addData("Simulated Distance", "%.1f in", testDistance);
            telemetry.addData("LUT Base Speed", "%.1f RPM", baseSpeed);
            telemetry.addData("Manual Speed Offset", "%.1f RPM", velocityOffset);
            telemetry.addData("COMBINED TARGET SPEED", "%.1f RPM", finalTargetSpeed);
            telemetry.addLine("------------------------");
            telemetry.addData("LUT Target Hood Position Data", "%.3f", targetAngle);
            telemetry.addData("Actual Hood Servo Commanded Pos", "%.3f", shooter.getHoodPosition());
            telemetry.addLine("------------------------");
            telemetry.addData("Actual Avg Speed", "%.1f RPM", shooter.getAverageVelocity());
            telemetry.addData("Shooter State", shooter.getState());
            telemetry.update();
        }
        shooter.requestStop();
    }
}