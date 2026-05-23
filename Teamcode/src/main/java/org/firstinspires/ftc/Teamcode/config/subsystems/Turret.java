package org.firstinspires.ftc.Teamcode.config.subsystems;

public class Turret {
    // These come from your goBILDA 3-wheel tracker loop
    // (Note: we assume the tracker supplies X/Y in inches, and Heading in RADIANS)
    private double robotX, robotY, robotHeadingRadians;

    // Constants for the high goal target (in field inches)
    public static final double GOAL_X = -72.0;
    public static final double GOAL_Y = 36.0;

    // Tuning constant for proportional rotation speed control
    // Adjusted lower because our calculations now use RADIANS for error processing
    public static double kP_Turn = 0.5;

    /**
     * Updates the subsystem with the latest goBILDA tracking coordinates.
     * @param x Current X position in inches
     * @param y Current Y position in inches
     * @param headingRadians Current angle in Radians (-pi to +pi)
     */
    public void updatePose(double x, double y, double headingRadians) {
        this.robotX = x;
        // goBILDA and standard FTC field maps invert the Y axis relative to Cartesian math.
        // We invert it here so the trigonometric atan2 function calculates correctly.
        this.robotY = -y;
        this.robotHeadingRadians = headingRadians;
    }

    /**
     * Calculates the absolute angle from the robot's current position to the target.
     * @return Target angle in RADIANS
     */
    public double getTargetAngle() {
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;

        // Math.atan2 natively outputs in Radians, matching goBILDA's data structure
        return Math.atan2(deltaY, deltaX);
    }

    /**
     * Calculates the direct angular distance to the target.
     * @return Normalized tracking error in RADIANS (-pi to +pi)
     */
    public double getAimError() {
        double target = getTargetAngle();
        double error = target - robotHeadingRadians;

        // Normalize the angle using Radians (PI) so the turret takes the shortest path
        while (error > Math.PI)  error -= (2 * Math.PI);
        while (error <= -Math.PI) error += (2 * Math.PI);

        return error;
    }

    /**
     * Calculates the real-time proportional motor power output.
     * @return Capped motor power scaling from -0.5 to 0.5
     */
    public double calculateAutoAimPower() {
        double error = getAimError();

        // Generates proportional correction power
        double power = error * kP_Turn;

        // Safety cap to prevent violent turret velocity spikes
        return Math.max(-0.5, Math.min(0.5, power));
    }
}
