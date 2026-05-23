package org.firstinspires.ftc.Teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.Teamcode.pedroPathing.Close18Ball;

@Autonomous(name = "Decode 18 Ball Auto", group = "Autonomous")
public class DecodeAuto extends OpMode {
    private Follower follower;
    private Paths myPaths;
    private Timer pathTimer;
    private int pathState = 0;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        myPaths = new Paths(follower);
        pathTimer = new Timer();
        
        // Path 1 start coordinates
        follower.setStartingPose(new Pose(27.149, 134.762, Math.toRadians(142)));
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0: // Start Path 1
                follower.followPath(myPaths.Path1);
                pathState = 1;
                break;

            case 1: // Wait 1500ms after Path 1
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeMS() > 1500) {
                        follower.followPath(myPaths.Path2);
                        pathState = 2;
                        pathTimer.resetTimer();
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 2: // Path 2 (No wait)
                if (!follower.isBusy()) {
                    follower.followPath(myPaths.Path3);
                    pathState = 3;
                }
                break;

            case 3: // Path 3 (No wait)
                if (!follower.isBusy()) {
                    follower.followPath(myPaths.Path4);
                    pathState = 4;
                }
                break;

            case 4: // Wait 1500ms after Path 4
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeMS() > 1500) {
                        follower.followPath(myPaths.Path5);
                        pathState = 5;
                        pathTimer.resetTimer();
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 5: // Path 5 (No wait)
                if (!follower.isBusy()) {
                    follower.followPath(myPaths.Path6);
                    pathState = 6;
                }
                break;

            case 6: // Path 6 (No wait)
                if (!follower.isBusy()) {
                    follower.followPath(myPaths.Path7);
                    pathState = 7;
                }
                break;

            case 7: // Wait 1500ms after Path 7
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeMS() > 1500) {
                        follower.followPath(myPaths.Path8);
                        pathState = 8;
                        pathTimer.resetTimer();
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 8: // Wait 1500ms after Path 8
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeMS() > 1500) {
                        follower.followPath(myPaths.Path9);
                        pathState = 9;
                        pathTimer.resetTimer();
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 9: // Wait 1500ms after Path 9
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeMS() > 1500) {
                        follower.followPath(myPaths.Path10);
                        pathState = 10;
                        pathTimer.resetTimer();
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 10: // Wait 1500ms after Path 10
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeMS() > 1500) {
                        follower.followPath(myPaths.Path11);
                        pathState = 11;
                        pathTimer.resetTimer();
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 11: // Wait 1500ms after Path 11
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeMS() > 1500) {
                        follower.followPath(myPaths.Path12);
                        pathState = 12;
                        pathTimer.resetTimer();
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 12: // Path 12 (No wait)
                if (!follower.isBusy()) {
                    follower.followPath(myPaths.Path13);
                    pathState = 13;
                }
                break;

            case 13: // Wait 1500ms after Path 13
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeMS() > 1500) {
                        follower.followPath(myPaths.Path14);
                        pathState = 14;
                        pathTimer.resetTimer();
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 14: // Final Path
                if (!follower.isBusy()) {
                    pathState = -1; // Done!
                }
                break;
        }

        telemetry.addData("Current State", pathState);
        telemetry.update();
    }
}
