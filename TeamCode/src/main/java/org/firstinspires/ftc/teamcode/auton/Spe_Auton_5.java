package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.SpecMec;
import org.firstinspires.ftc.teamcode.subsystems.Util;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Config
@Autonomous
public class Spe_Auton_5 extends OpMode {
    private Follower follower;
    private Util util = new Util();

    private Timer pathTimer, actionTimer, opmodeTimer;
    private Pivot pivot;
    private SpecMec specMec;

    public static double waitTime = 0.5;
    private Wrist wrist;

    public static double hang0X = 36.25, hangX = 39,pickX = 13.5, pickY = 31, hangY = 74, blockX = 29, block3X = 18, block3Y = 10, blockY = 23, block2Y = 15, pushControlX = 63, parkX = 30, parkY = 25, pickX4 = 13.5, pickX3 = 14.5, hang3XChange = -8, hang4XChange = -10, pickY1 = 31;

    public static int pivotDownTime = 0, idleTime0 = 0, scoreTime0 = 1100, openTime0 = 1800 , closeTime1 = 500;

    public static double pullOutPar = 0.1, idlePar = 0, scorePar = 0.93, openPar = 0.97, closePar = 0.9, specMecDownPar = 0, specMecParkPar = 0, scorePar4 = 0.95, openPar4 = 0.99, scorePar3 = 0.95, openPar3 = 0.99;

    public static double push3Timout = 2.5;
    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    private final Pose startPose = new Pose(8, 64, Math.toRadians(270));

    private final Pose hang0Pose = new Pose(hang0X, hangY, Math.toRadians(0));

    private final Pose pullOutPose = new Pose(43, 36, Math.toRadians(0));

    private final Pose push1Pose = new Pose(blockX, blockY, Math.toRadians(0));
    private final Pose push2Pose = new Pose(blockX, block2Y, Math.toRadians(0));
    private final Pose push3Pose = new Pose(block3X, block3Y, Math.toRadians(0));

    private final Pose pickupPose = new Pose(pickX, pickY, Math.toRadians(0));
    private final Pose pickupPose1 = new Pose(pickX, pickY1, Math.toRadians(0));

    private final Pose pickupPose3 = new Pose(pickX3, pickY, Math.toRadians(0));
    private final Pose pickupPose4 = new Pose(pickX4, pickY, Math.toRadians(0));

    private final Pose hang1Pose = new Pose(hangX, hangY-2, Math.toRadians(0));
    private final Pose hang2Pose = new Pose(hangX, hangY-4, Math.toRadians(0));
    private final Pose hang3Pose = new Pose(hangX, hangY+hang3XChange, Math.toRadians(0));
    private final Pose hang4Pose = new Pose(hangX, hangY+hang4XChange, Math.toRadians(0));

    private final Pose parkPose = new Pose(parkX-15, parkY, Math.toRadians(0));

    private final Pose hangControl1 = new Pose(30.592692828146145, 31.7618403247632, Math.toRadians(0));
    private final Pose hangControl2 = new Pose(22.99323410013532, 74.8254397834912, Math.toRadians(0));
    private final Pose hangControl23 = new Pose(22.99323410013532, 74.8254397834912 + hang3XChange, Math.toRadians(0));
    private final Pose hangControl24 = new Pose(22.99323410013532, 74.8254397834912 + hang4XChange, Math.toRadians(0));


    private PathChain hangPreload, pushBlocks, pick1, hang1, pick2, hang2, pick3, hang3, pick4, hang4, park;

    public void buildPaths() {
        hangPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose), new Point(hang0Pose)
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), hang0Pose.getHeading())
                .addTemporalCallback(idleTime0, () -> specMec.setPosition("Idle", "Score"))
                .addTemporalCallback(scoreTime0, () -> specMec.setPosition("Score", "Score"))
                .addTemporalCallback(openTime0, () -> specMec.openClaw())
                .build();

        pushBlocks = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(hang0Pose),
                                new Point(10.522327469553451, 46.765899864682005, Point.CARTESIAN),
                                new Point(pullOutPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(pullOutPar, () -> specMec.setPosition("Intake", "Intake"))
                .addPath(
                        new BezierCurve(
                                new Point(pullOutPose),
                                new Point(pushControlX, 33.71041948579162, Point.CARTESIAN),
                                new Point(70.344, 25.916, Point.CARTESIAN),
                                new Point(push1Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(push1Pose),
                                new Point(pushControlX, 26.696, Point.CARTESIAN),
                                new Point(87.296, 16.173, Point.CARTESIAN),
                                new Point(push2Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(push2Pose),
                                new Point(pushControlX, 15.004, Point.CARTESIAN),
                                new Point(72.68200270635995, 6.820027063599452, Point.CARTESIAN),
                                new Point(push3Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(push3Timout)
                .build();

        pick1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(push3Pose),
                                new Point(27.66982408660352, 32, Point.CARTESIAN),
                                new Point(pickupPose1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(closeTime1, () -> specMec.closeClaw())
                .build();

        hang1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickupPose),
                                new Point(hangControl1),
                                new Point(hangControl2),
                                new Point(hang1Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(idlePar, () -> specMec.setPosition("Idle", "Score"))
                .addParametricCallback(scorePar, () -> specMec.setPosition("Score", "Score"))
                .addParametricCallback(openPar, () -> specMec.openClaw())
                .build();

        pick2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(hang1Pose),
                                new Point(hangControl2),
                                new Point(hangControl1),
                                new Point(pickupPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(specMecDownPar, () -> specMec.setPosition("Intake", "Intake"))
                .addParametricCallback(closePar, () -> specMec.closeClaw())
                .build();

        hang2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickupPose),
                                new Point(hangControl1),
                                new Point(hangControl2),
                                new Point(hang2Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(idlePar, () -> specMec.setPosition("Idle", "Score"))
                .addParametricCallback(scorePar, () -> specMec.setPosition("Score", "Score"))
                .addParametricCallback(openPar, () -> specMec.openClaw())
                .build();

        pick3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(hang3Pose),
                                new Point(hangControl2),
                                new Point(hangControl1),
                                new Point(pickupPose3)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(specMecDownPar, () -> specMec.setPosition("Intake", "Intake"))
                .addParametricCallback(closePar, () -> specMec.closeClaw())
                .build();

        hang3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickupPose3),
                                new Point(hangControl1),
                                new Point(hangControl23),
                                new Point(hang3Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(idlePar, () -> specMec.setPosition("Idle", "Score"))
                .addParametricCallback(scorePar3, () -> specMec.setPosition("Score", "Score"))
                .addParametricCallback(openPar3, () -> specMec.openClaw())
                .build();

        pick4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(hang3Pose),
                                new Point(hangControl2),
                                new Point(hangControl1),
                                new Point(pickupPose4)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(specMecDownPar, () -> specMec.setPosition("Intake", "Intake"))
                .addParametricCallback(closePar, () -> specMec.closeClaw())
                .build();

        hang4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickupPose4),
                                new Point(hangControl1),
                                new Point(hangControl24),
                                new Point(hang4Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(idlePar, () -> specMec.setPosition("Idle", "Score"))
                .addParametricCallback(scorePar4, () -> specMec.setPosition("Score", "Score"))
                .addParametricCallback(openPar4, () -> specMec.openClaw())
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(hang4Pose),
                                new Point(20, 63, Point.CARTESIAN),
                                new Point(parkPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(pivotDownTime, () -> pivot.setPos("Down"))
                .addParametricCallback(specMecParkPar, () -> specMec.setPosition("Intake", "Intake"))
                .addParametricCallback(specMecParkPar, () -> specMec.closeClaw())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(hangPreload);
                setPathState(1);
                break;
            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(pushBlocks, true);
                    setPathState(2);
                }
                break;
            case 2: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(pick1, true);
                    setPathState(3);
                }
                break;
            case 3: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(hang1, true);
                    setPathState(4);
                }
                break;
            case 4: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(pick2, true);
                    setPathState(5);
                }
                break;
            case 5: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(hang2, true);
                    setPathState(6);
                }
                break;
            case 6: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(pick3, true);
                    setPathState(7);
                }
                break;
            case 7: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(hang3, true);
                    setPathState(8);
                }
                break;
            case 8: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(pick4, true);
                    setPathState(9);
                }
                break;
            case 9: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(hang4, true);
                    setPathState(10);
                }
                break;
            case 10: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pivot = new Pivot(hardwareMap, util.deviceConf);
        specMec = new SpecMec(hardwareMap, util.deviceConf);
        wrist = new Wrist(hardwareMap, util.deviceConf);
        pathTimer = new Timer();
        pivot.setPos("Start");
        wrist.setPos("Start");
        specMec.setPosition("Start", "Start");
        specMec.closeClaw();
        wrist.setRotationPos(0);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        pivot.update();
        specMec.update();
        specMec.updateClaw();
        wrist.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("pos", pivot.getTarget());
        telemetry.addData("pos", pivot.getPower());
        telemetry.update();
        follower.drawOnDashBoard();
    }

    @Override
    public void start() {
        wrist.setPos("Start");
        pivot.setPos("Basket");
        specMec.setPosition("Idle", "Score");
        pathTimer.resetTimer();
        while (pathTimer.getElapsedTimeSeconds() < waitTime)
        {
            telemetry.addLine("wating");
        }
        setPathState(0);
    }

    @Override
    public void init_loop() {
        wrist.update();
        pivot.update();
        specMec.update();
        specMec.updateClaw();
    }
    


}
