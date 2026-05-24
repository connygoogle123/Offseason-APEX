package org.firstinspires.ftc.teamcode.config.subsystems;

public class ShooterTable {

    public static class TuningTable {
        public double p;
        public double velocity;
        public double f;


        public TuningTable(double velocity, double p, double f) {
            this.p = p;
            this.f = f;
            this.velocity = velocity;
        }
    }
    public ShooterTable() {

    }
}
