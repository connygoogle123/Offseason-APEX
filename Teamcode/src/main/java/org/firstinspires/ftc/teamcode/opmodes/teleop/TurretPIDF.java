package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

@TeleOp (name = "Turret Only Tuning", group = "TeleOp")
public class TurretPIDF extends LinearOpMode {

    private Turret turretSubsystem;
    private boolean lastRightBumperState = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;

    @Override
    public void runOpMode() throws InterruptedException {
        turretSubsystem = new Turret(hardwareMap);

        telemetry.addLine("Ready! Point turret straight forward before hitting INIT.");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            // Toggle lock (GamePad 1 Right BUMPER)
            if (gamepad1.right_bumper && !lastRightBumperState) {
                turretSubsystem.setAutoAimEnabled(!turretSubsystem.isAutoAimEnabled());
            }


            turretSubsystem.update();

            double currentDeg = turretSubsystem.turret.getCurrentPosition() / 5.37;

            telemetry.addLine("=== PID TUNING STEP ===");
            telemetry.addData("Auto-Aim Enabled", turretSubsystem.isAutoAimEnabled() ? "ACTIVE (Locking to 0°)" : "OFF");
            telemetry.addData("CURRENT kP VALUE", "%.4f", turretSubsystem.turret_kP);
            telemetry.addData("Current Turret Pos", "%.2f°", currentDeg);
            telemetry.update();
        }
    }
}