package org.firstinspires.ftc.teamcode.config.tests;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class shootertest {
    private static final InterpLUT angleILUT = new InterpLUT();
    private static final InterpLUT speedILUT = new InterpLUT();

    public static double getSpeed(double dist) {
        return speedILUT.get(dist);
    }

    public static double getAngle(double dist) {
        return angleILUT.get(dist);
    }

    static {
        // X MUST BE INCREASING ORDER WHEN ADDING
        // otherwise it literally crashes the entire robot with no error message so dont do it

        // Boundary safety buffers
        angleILUT.add(-500, 0);
        speedILUT.add(-500, 1250);


        // Measured tuning data points (Mapped to Desmos coords)
        angleILUT.add(40, 104);
        speedILUT.add(40, 1350);

        angleILUT.add(50, 160);
        speedILUT.add(50, 1450);

        angleILUT.add(60, 190);
        speedILUT.add(60, 1550);

        angleILUT.add(67, 215);
        speedILUT.add(67, 1550);

        angleILUT.add(70, 225);
        speedILUT.add(70, 1600);

        angleILUT.add(80, 270);
        speedILUT.add(80, 1650);

        angleILUT.add(90, 290);
        speedILUT.add(90, 1750);

        angleILUT.add(100, 225);
        speedILUT.add(100, 1850);

        angleILUT.add(148, 245);
        speedILUT.add(148, 2050);

        angleILUT.add(157, 255);
        speedILUT.add(157, 2100);

        // Far boundary safety buffers
        angleILUT.add(500, 225);
        speedILUT.add(500, 2100);

        // Finalize look-up tables
        speedILUT.createLUT();
        angleILUT.createLUT();
    }
}