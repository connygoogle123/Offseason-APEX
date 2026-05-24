package org.firstinspires.ftc.Teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.Teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.Teamcode.config.subsystems.RGB;
import org.firstinspires.ftc.Teamcode.config.subsystems.Shooter;


@TeleOp (name = "MainTeleOp", group = "TeleOp")
public class Tele extends LinearOpMode {

    private Intake intake;
    private Shooter shooter;

    public void runOpMode() throws InterruptedException {

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {


            if (gamepad2.left_trigger > 0.25) {
                intake.intake();

            } else if (gamepad2.left_bumper) {
                intake.reverse();

            } else intake.stop();

            if (gamepad2.right_bumper) {
                shooter.requestSpinUp(1600);
            }
            if (gamepad2.x) {
                shooter.requestFeed();
            }
            if (gamepad2.a) {
                shooter.requestStop();
            }

            shooter.update();
            intake.updateLight();
            intake.update();

            telemetry.addData("Intake State: ", intake.getState());
            telemetry.addData("Current Amps: ", intake.getCurrent());
            telemetry.addData("Estimated Balls: ", intake.getBallCount());
            telemetry.update();

        }
    }
}
