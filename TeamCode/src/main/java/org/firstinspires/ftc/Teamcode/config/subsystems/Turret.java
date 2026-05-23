package org.firstinspires.ftc.Teamcode.config.subsystems;
  
  public class Turret {
    // These would come from your Odometry system (like Road Runner or a custom tracker)
    double robotX, robotY, robotHeading; 

    // Constants for the goal (example: Center of the high goal)
    public static final double GOAL_X = 72.0; 
    public static final double GOAL_Y = 36.0;

    // Tuning constant for rotation speed
    public static double kP_Turn = 0.015; 

    public double getTargetAngle() {
        // Calculate the distance to target
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;

        // Math.atan2 returns radians, we convert to degrees
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        
        return angleToTarget;
    }

    public double getAimError() {
        double target = getTargetAngle();
        double error = target - robotHeading;

        // "Normalize" the angle so the robot doesn't spin 350 degrees 
        // to turn 10 degrees the other way
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        return error;
    }

    public double calculateAutoAimPower() {
        double error = getAimError();
        
        // If error is 20 degrees, power is 20 * 0.015 = 0.3
        double power = error * kP_Turn;

        // Cap the power so the robot doesn't spin out of control
        return Math.max(-0.5, Math.min(0.5, power));
    }
}

