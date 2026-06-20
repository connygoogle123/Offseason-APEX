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
        // X MUST BE IN INCREASING ORDER WHEN ADDING
        // Otherwise it will crash the entire robot loop.

        // Boundary safety buffers
        angleILUT.add(-500, 0);
        speedILUT.add(-500, 1250);

        // Measured tuning data points (Mapped to Desmos coords)
        // Adjust the '0' values to your baseline angle once found!
        angleILUT.add(40, 0);
        speedILUT.add(40, 1330);

        angleILUT.add(50, 0);
        speedILUT.add(50, 1430);

        angleILUT.add(60, 0);
        speedILUT.add(60, 1590);

        angleILUT.add(70, 0);
        speedILUT.add(70, 1635);

        angleILUT.add(75, 28);
        speedILUT.add(75, 1640);

        angleILUT.add(80, 31.5);
        speedILUT.add(80, 1640);

        angleILUT.add(90, 35);
        speedILUT.add(90, 1730);

        angleILUT.add(100, 34);
        speedILUT.add(100, 1830);

        angleILUT.add(150, 46);
        speedILUT.add(150, 2230);

        // Far boundary safety buffers
        angleILUT.add(500, 225);
        speedILUT.add(500, 2300);

        // Finalize look-up tables
        speedILUT.createLUT();
        angleILUT.createLUT();
    }
}