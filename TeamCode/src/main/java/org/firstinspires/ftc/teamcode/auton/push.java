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
public class push extends OpMode {
    private Follower follower;
    private Util util = new Util();

    private Timer pathTimer, actionTimer, opmodeTimer;
    private Pivot pivot;
    private SpecMec specMec;

    public static double waitTime = 0.5;

    public static double dist = 3;
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
    private final Pose endPose = new Pose(8, 64+dist, Math.toRadians(270));



    private PathChain hangPreload, pushBlocks, pick1, hang1, pick2, hang2, pick3, hang3, pick4, hang4, park;

    public void buildPaths() {
        hangPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose), new Point(endPose)
                        )
                )
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(hangPreload);
                setPathState(1);
                break;

            case 1:
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
