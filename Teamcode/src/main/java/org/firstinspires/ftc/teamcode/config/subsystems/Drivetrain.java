package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private double driveSpeedMultiplier = 1.0; // full speed 312

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");

        // Reverse the left side or right side depending on drivetrain
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void drive(double forward, double strafe, double rotate) {
        // the denominator is the largest motor power (absolute value) or 1
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double frontLeftPower = (forward + strafe + rotate) / denominator;
        double frontRightPower = (forward - strafe - rotate) / denominator;
        double backLeftPower = (forward - strafe + rotate) / denominator;
        double backRightPower = (forward + strafe - rotate) / denominator;

        // Apply MaxSpeed scaling
        frontLeft.setPower(frontLeftPower * driveSpeedMultiplier);
        frontRight.setPower(frontRightPower * driveSpeedMultiplier);
        backLeft.setPower(backLeftPower * driveSpeedMultiplier);
        backRight.setPower(backRightPower * driveSpeedMultiplier);
    }
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void setDriveSpeedMultiplier(double multiplier) {
        driveSpeedMultiplier = Math.max(0.0, Math.min(1.0, multiplier));
    }

    public double getDriveSpeedMultiplier() {
        return driveSpeedMultiplier;
    }
}
