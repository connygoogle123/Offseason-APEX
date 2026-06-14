package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

@TeleOp(name = "Main TeleOp: With Shooter", group = "Linear OpMode")
public class Tele extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Turret turret;
    private Shooter shooter;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        telemetry.addData("Status", "Initializing Subsystems...");
        telemetry.update();

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

        turret.setAutoAimEnabled(true);

        telemetry.addData("Status", "Initialized! Ready to Start.");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        // Main Teleop
        while (opModeIsActive()) {
            double loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();

            //dirvetrain stuff
            double forward = gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double rotate  = gamepad1.right_stick_x;

            if (gamepad2.right_bumper) {
                drivetrain.setDriveSpeedMultiplier(1.00);
            } else {
                drivetrain.setDriveSpeedMultiplier(0.70);
            }

            drivetrain.drive(forward, strafe, rotate);

            // Intake

            if (gamepad2.right_trigger > 0.1) {
                intake.intake();
            } else if (gamepad2.left_trigger > 0.1) {
                intake.reverse();
            } else {
                intake.stop();
            }
            intake.update();

            // Shooter, currently not working
            // Press A to spin up
            if (gamepad2.a) {

                shooter.aimForDistance(102.0);
                shooter.requestSpinUp(shooter.getTargetVelocity());
            }
            // Press B to shut down flywheels
            else if (gamepad2.b) {
                shooter.requestStop();
            }

            // Press Right bumper to fire once flywheels ready
            if (gamepad2.right_bumper) {
                shooter.requestFeed();
            }

            // shooter override, never tested before
            if (shooter.shouldRunTransfer()) {
                intake.intake();
            }

            shooter.update();

            // turret background calculations
            if (gamepad2.y) {
                turret.setAutoAimEnabled(true);
            } else if (gamepad2.x) {
                turret.setAutoAimEnabled(false);
            }
            turret.update();

            // telemetry

            telemetry.addData("Loop Speed (Hz)", (loopTimeMs > 0) ? "%.1f Hz" : "0.0 Hz", (1000.0 / loopTimeMs));

            telemetry.addData("--- SHOOTER ---", "");
            telemetry.addData("State", shooter.getState());
            telemetry.addData("Target Velocity", "%.1f", shooter.getTargetVelocity());
            telemetry.addData("Current Velocity", "%.1f", shooter.getAverageVelocity());
            telemetry.addData("At Speed?", shooter.atSpeed() ? "READY" : "SPINNING UP");

            telemetry.update();
        }

        drivetrain.stop();
        intake.stop();
        turret.setAutoAimEnabled(false);
        shooter.requestStop();
    }
}