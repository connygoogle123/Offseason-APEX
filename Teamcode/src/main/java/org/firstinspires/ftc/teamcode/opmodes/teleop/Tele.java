package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

@TeleOp(name = "Main TeleOp", group = "Linear OpMode")
public class Tele extends LinearOpMode {

    // subs
    private Drivetrain drivetrain;
    private Intake intake;
    private Turret turret;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---
        telemetry.addData("Status", "Initializing Subsystems...");
        telemetry.update();

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

        // Turn on turret tracking from start
        turret.setAutoAimEnabled(true);

        telemetry.addData("Status", "Initialized! Ready to Start.");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        // main teleop
        while (opModeIsActive()) {
            double loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();

            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double rotate  = gamepad1.right_stick_x;

            // Optional precision speed toggle (Hold Right Bumper for slow mode)
            if (gamepad1.right_bumper) {
                drivetrain.setDriveSpeedMultiplier(0.40);
            } else {
                drivetrain.setDriveSpeedMultiplier(1.00);
            }

            drivetrain.drive(forward, strafe, rotate);


            if (gamepad1.right_trigger > 0.1) {
                intake.intake();
            } else if (gamepad1.left_trigger > 0.1) {
                intake.reverse();
            } else {
                intake.stop();
            }

            // Execute intake voltage outputs
            intake.update();


            if (gamepad1.y) {
                turret.setAutoAimEnabled(true);
            } else if (gamepad2.x) {
                turret.setAutoAimEnabled(false);
            }

            // pinpoint stuff
            turret.update();

            // ==========================================
            // 4. TELEMETRY DIAGNOSTICS
            // ==========================================
            telemetry.addData("--- SYSTEM DIAGNOSTICS ---", "");
            telemetry.addData("Loop Speed (Hz)", "%.1f Hz", (1000.0 / loopTimeMs));

            telemetry.addData("--- INTAKE STATS ---", "");
            telemetry.addData("Intake Current (Amps)", "%.2f A", intake.getCurrent());
            telemetry.addData("Balls Inside", "%d / 3", intake.getBallCount());
            telemetry.addData("State", intake.getState());

            telemetry.addData("--- TURRET STATS ---", "");
            telemetry.addData("Auto-Aim Status", turret.isAutoAimEnabled() ? "ACTIVE" : "DISABLED");
            telemetry.addData("Encoder Reading (Pos)", turret.turret.getCurrentPosition());

            telemetry.update();
        }

        // stop
        drivetrain.stop();
        intake.stop();
        turret.setAutoAimEnabled(false);
    }
}