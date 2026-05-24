package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.AutoAim;
@TeleOp (name = "MainTeleOp", group = "TeleOp")
public class Tele extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    // --- TOGGLE STATE MEMORY FOR AUTO AIM ---
    private boolean lastRightBumperState = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            // Variables that continuously change
            double forward, right, rotate;
            forward = -gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            rotate  =  gamepad1.right_stick_x;

            drivetrain.drive(forward, right, rotate);

            // --- AUTO AIM TOGGLE LOGIC ---
            // Drivers click Right Bumper on Gamepad 1 once to flip the tracking loop on/off
            if (gamepad1.right_bumper && !lastRightBumperState) {
                shooter.setAutoAimEnabled(!shooter.isAutoAimEnabled());
            }
            lastRightBumperState = gamepad1.right_bumper; // Save button state for next loop iteration


            if (gamepad2.left_trigger > 0.2) {
                intake.intake();
                shooter.gate.setPosition(shooter.gateClosed);
            } else if (gamepad2.left_bumper) {
                intake.reverse();
            } else {
                intake.stop();
            }

            if (gamepad2.x) {
                shooter.requestSpinUp(2000);
            }
            if (gamepad2.right_bumper) {
                shooter.requestFeed();
            }
            if (gamepad2.a) {
                shooter.requestStop();
            }

            // This line executes the flywheel control, servo logic, AND the auto-aim PID math
            shooter.update();

            if (shooter.getState() == Shooter.ShooterState.FEEDING) {
                intake.intake();
            }

            intake.update();
            intake.updateLight();

            // --- TELEMETRY ---
            telemetry.addData("Intake State: ", intake.getState());
            telemetry.addData("Current Amps: ", intake.getCurrent());
            telemetry.addData("Estimated Balls: ", intake.getBallCount());

            telemetry.addData("Auto-Aim Tracking: ", shooter.isAutoAimEnabled() ? "ON [LOCK]" : "OFF");
            telemetry.addData("State", shooter.getState());
            telemetry.addData("Left Velocity", shooter.getLeftVelocity());
            telemetry.addData("Right Velocity", shooter.getRightVelocity());
            telemetry.addData("Target Velocity: ", shooter.getTargetVelocity());
            telemetry.addData("Current Velocity (AV)", "%.2f", shooter.getAverageVelocity());
            telemetry.addData("Error", "%.2f", shooter.getTargetVelocity() - shooter.getAverageVelocity());
            telemetry.update();
        }
    }
}