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

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.SpecMec;
import org.firstinspires.ftc.teamcode.subsystems.Util;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Config
@Autonomous
public class sample_1 extends OpMode {
    private Follower follower;
    private Util util = new Util();

    private Timer pathTimer, actionTimer, opmodeTimer;
    private Pivot pivot;

    private Extension extension;

    private Wrist wrist;

    private SpecMec specMec;

    private Claw claw;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    public static double wait = 500;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    private final Pose startPose = new Pose(8, 108, Math.toRadians(270));

    private final Pose dropPose = new Pose(8, 120, Math.toRadians(270));
    private final Pose parkPos = new Pose(8, 108, Math.toRadians(270));

    private double extensionUpTime = 600, dropTime = 100, extensionDownTime = 500, pivotDownTime = 1000;

    private PathChain drop, park;

    public void buildPaths() {
        drop = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose), new Point(dropPose)
                        )
                )
                .addTemporalCallback(0, () -> pivot.setPos("Basket"))
                .addTemporalCallback(0, () -> wrist.setPos("Basket"))
                .addTemporalCallback(extensionUpTime, () -> extension.setPos("Basket"))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(dropPose), new Point(parkPos)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(pivotDownTime, () -> pivot.setPos("Down"))
                .addParametricCallback(extensionDownTime, () -> extension.setPos("Idle"))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(drop);
                setPathState(1);
                break;
            case 1: // Wait until the robot is near the scoring position
                if (pathTimer.getElapsedTime() > wait)
                {
                    claw.directSet(Claw.open);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(3);
                }
                break;
            case 3:
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
        extension = new Extension(hardwareMap, util.deviceConf);
        claw = new Claw(hardwareMap, util.deviceConf);
        pathTimer = new Timer();
        pivot.setPos("Start");
        wrist.setPos("Start");
        specMec.setPosition("Start", "Start");
        claw.directSet(Claw.closed);
        specMec.closeClaw();
        wrist.setRotationPos(0);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        /*pivot.update();
        specMec.update();
        specMec.updateClaw();
        wrist.update();
        extension.update();*/
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
        setPathState(0);
    }

    @Override
    public void init_loop() {
        /*wrist.update();
        pivot.update();
        specMec.update();
        specMec.updateClaw();
        extension.update();*/
    }



}
