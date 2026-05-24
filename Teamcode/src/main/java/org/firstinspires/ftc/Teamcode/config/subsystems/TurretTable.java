package org.firstinspires.ftc.Teamcode.config.subsystems;

public class TurretTable {

    public static class TurretTuning {

        public double p;
        public double velocity;
        public double f;

        public TurretTuning (double p, double f, double velocity) {
            this.p = p;
            this.f = f;
            this.velocity = velocity;

        }
    }
}
