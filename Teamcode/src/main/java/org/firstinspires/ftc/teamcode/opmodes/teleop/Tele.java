package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

@TeleOp(name = "MainTeleOp", group = "Competition")
public class Tele extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;

    private final double GOAL_X = 0.0;
    private final double GOAL_Y = 0.0;

    private boolean lastDpadUp = false;
    private boolean lastGamepad1X = false;
    private boolean isGateToggledOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        telemetry.addLine("Systems Anchored. Ready to initialize match...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double rotate  = gamepad1.right_stick_x;
            drivetrain.drive(forward, strafe, rotate);

            if (gamepad1.dpad_up && !lastDpadUp) {
                turret.setAutoAimEnabled(!turret.isAutoAimEnabled());
            }
            lastDpadUp = gamepad1.dpad_up;
            turret.update();

            double robotX = turret.pinpoint.getPosX(DistanceUnit.INCH);
            double robotY = turret.pinpoint.getPosY(DistanceUnit.INCH);

            double deltaX = GOAL_X - robotX;
            double deltaY = GOAL_Y - robotY;
            double calculatedDistance = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));

            shooter.aimForDistance(calculatedDistance);

            if (gamepad1.x && !lastGamepad1X) {
                isGateToggledOpen = !isGateToggledOpen;
                if (isGateToggledOpen) {
                    shooter.openGate();
                } else {
                    shooter.closeGate();
                }
            }
            lastGamepad1X = gamepad1.x;

            if (gamepad1.a) {
                shooter.requestSpinUp(shooter.getTargetVelocity());
                intake.intake();
            }
            else if (gamepad1.right_bumper) {
                intake.intake();
            }
            else if (gamepad1.left_bumper) {
                intake.reverse();
            }
            else if (gamepad1.b) {
                shooter.requestStop();
                intake.stop();
                shooter.closeGate();
                isGateToggledOpen = false;
            }
            else {
                intake.stop();
                if (!gamepad1.a && !isGateToggledOpen) {
                    shooter.closeGate();
                }
            }

            shooter.update();
            intake.update(); // FIXED: Runs state machine logic to pass power to physical motors
            intake.updateLight();

            telemetry.addLine("=== OPERATIONAL MATRIX ===");
            telemetry.addData("Calculated Goal Distance", "%.2f inches", calculatedDistance);
            telemetry.addData("Live Target Speed", "%.1f RPM", shooter.getTargetVelocity());
            telemetry.addData("Actual Wheel Speed", "%.1f RPM", shooter.getAverageVelocity());
            telemetry.addData("Gate Position", shooter.gate.getPosition() == shooter.gateOpen ? "OPEN" : "CLOSED");
            telemetry.update();
        }

        shooter.requestStop();
        drivetrain.stop();
        intake.stop();
        turret.setAutoAimEnabled(false);
    }
}