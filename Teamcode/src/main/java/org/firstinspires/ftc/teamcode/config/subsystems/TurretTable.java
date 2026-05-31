package org.firstinspires.ftc.teamcode.config.subsystems;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class TurretTable {
    private static final InterpLUT angleILUT = new InterpLUT();
    private static final InterpLUT speedILUT = new InterpLUT();

    public static double getSpeed(double dist) {
        return speedILUT.get(dist);
    }

    public static double getAngle(double dist) {
        return angleILUT.get(dist);
    }
    static {
        angleILUT.add(60,)
        speedILUT.add(60,)

        angleILUT.add(60,)
        speedILUT.add(60,)

        angleILUT.add(60,)
        speedILUT.add(60,)

        angleILUT.add(60,)
        speedILUT.add(60,)

        angleILUT.add(60,)
        speedILUT.add(60,)

        angleILUT.add(60,)
        speedILUT.add(60,)

        angleILUT.add(60,)
        speedILUT.add(60,)

        angleILUT.add(60,)
        speedILUT.add(60,)


        speedILUT.createLUT();
        angleILUT.createLUT();

    }
}