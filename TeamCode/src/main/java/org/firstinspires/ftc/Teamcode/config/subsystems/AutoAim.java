package org.firstinspires.ftc.Teamcode.config.subsystems;

public class AutoAim {
    /**
     * Calculates the motor power required to align an independent turret with a field target.
     *
     * @param robotX          Current X coordinate from your chassis odometry system
     * @param robotY          Current Y coordinate from your chassis odometry system
     * @param robotHeadingDeg Current chassis heading/angle in DEGREES from your odometry system
     * @param currentTurretDeg Current absolute position of the turret relative to the field (or relative to the robot)
     * @return Motor power for the turret rotation motor (ranging from -1.0 to 1.0)
     */
    public double calculateTurretAimPower(double robotX, double robotY, double robotHeadingDeg, double currentTurretDeg) {
        // 1. Define where the high goal is located on the FTC field coordinate grid
        final double GOAL_X = 72.0;
        final double GOAL_Y = 36.0;

        // 2. Proportional tuning constant for the turret motor (Adjust this based on gear ratio)
        final double kP_Turret = 0.03;

        // 3. Find the distance vectors between the robot and the target location
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;

        // 4. Calculate the absolute field angle to the target in degrees
        double targetFieldAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // 5. Calculate the rotational difference (error) between the turret's current angle and the target line-of-sight
        double turretError = targetFieldAngleDeg - currentTurretDeg;

        // 6. Normalize the error to ensure the turret takes the shortest rotation path (-180 to 180 degrees)
        while (turretError > 180)  turretError -= 360;
        while (turretError <= -180) turretError += 360;

        // 7. Scale the turret motor power output based on the size of the angle error
        double turretPower = turretError * kP_Turret;

        // 8. Clip the output power to safe operating speeds for your turret gearbox
        return Math.max(-0.6, Math.min(0.6, turretPower));
    }
}
