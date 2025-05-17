package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AutonPivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Util;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

import java.util.HashMap;
@Config
@Autonomous(name = "Sample Cycle (4)", group = "Sensor")
public class Sample_RR extends LinearOpMode {

    public static int tickChange = 100, pos = 150;
    public static double basketX = 58, basketY = 58, intakeY= 41, intake3X = 64.5, intake3Y = 40, basket0X = 58, basket0Y = 58, intakeX = 53, intake2X= 62, intake2Y = 42;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Map HashMap
        Util util = new Util();

        Claw claw = new Claw(hardwareMap, util.deviceConf);

        Pivot pivot = new Pivot(hardwareMap, util.deviceConf);

        Extension extension = new Extension(hardwareMap, util.deviceConf);

        Wrist wrist = new Wrist(hardwareMap, util.deviceConf);

        Pose2d startPos = new Pose2d(40.5, 66, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        TrajectoryActionBuilder bucket0 = drive.actionBuilder(startPos)
                .setTangent(3*Math.PI/2)
                .splineToLinearHeading(new Pose2d(basket0X,basket0Y, 5*Math.PI/4), 0);

        TrajectoryActionBuilder wait1 = drive.actionBuilder(startPos)
                .waitSeconds(1);


        TrajectoryActionBuilder block1 = bucket0.endTrajectory().fresh()
                .setTangent(5*Math.PI/4)
                .splineToLinearHeading(new Pose2d(intakeX, intakeY, 3*Math.PI/2), 3*Math.PI/2);

        TrajectoryActionBuilder bucket1 = block1.endTrajectory().fresh()
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(basketX,basketY, 5*Math.PI/4), Math.PI/4);


        TrajectoryActionBuilder block2 = bucket1.endTrajectory().fresh()
                .setTangent(3*Math.PI/2)
                .splineToLinearHeading(new Pose2d(intake2X, intake2Y, 3*Math.PI/2), 3*Math.PI/2);

        TrajectoryActionBuilder bucket2 = block2.endTrajectory().fresh()
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(basketX,basketY, 5*Math.PI/4), Math.PI/2);

        TrajectoryActionBuilder block3 = bucket2.endTrajectory().fresh()
                .setTangent(3*Math.PI/2)
                .splineToLinearHeading(new Pose2d(intake3X, intake3Y, 5*Math.PI/3), 5*Math.PI/3);

        TrajectoryActionBuilder bucket3 = block3.endTrajectory().fresh()
                .setTangent(2*Math.PI/3)
                .splineToLinearHeading(new Pose2d(basketX,basketY, 5*Math.PI/4), Math.PI/2);

        TrajectoryActionBuilder park = bucket2.endTrajectory().fresh()
                .strafeTo(new Vector2d(50, 50));

        Thread update = new Thread(()->updateAll(pivot, extension, wrist));


        // go to start pos
        //claw.directSet(Claw.closed);
        wrist.setBicepPos("Start");
        wrist.setForearmPos("Start");
        wrist.setRotationPos(0);
        pivot.setPos("Start");
        pivot.setkP("Normal");
        update.start();

        telemetry.addData("pos", pivot.getCurrent());
        telemetry.addData("target", pivot.getTarget());
        telemetry.addData("power", pivot.getPower());
        telemetry.addData("error", pivot.getError());
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        extension.setPos("Idle");


        setBucket(bucket0.build(), pivot, extension, wrist, claw);
        //sleep(500);

        getBlock(block1.build(),pivot,extension,wrist,claw);
        setBucket(bucket1.build(), pivot, extension, wrist, claw);
        //sleep(500);

        getBlock(block2.build(), pivot, extension, wrist, claw);
        setBucket(bucket2.build(), pivot, extension, wrist, claw);


        extension.setPos("Idle");


        Actions.runBlocking(block3.build());
        pivot.setkP("Normal");
        pivot.setPos("Down");

        wrist.setRotationPos(4);

        wrist.setPos("Intake");
        sleep(1500);
        //claw.directSet(Claw.closed);
        sleep(500);

        pivot.setPos("Basket");



        pivot.setPos("Basket");
        //pivot.setkP("Extended");
        sleep(1000);
        wrist.setPos("Auton Idle");
        wrist.setRotationPos(0);
        extension.setPos("Basket");

        Actions.runBlocking(bucket3.build());
        wrist.setPos("Basket");
        sleep(500);
        //claw.directSet(Claw.open);
        sleep(500);
        wrist.setPos("Auton Idle");
        sleep(500);


        extension.setPos("Idle");
        Actions.runBlocking(park.build());


        pivot.setPos("Down");

        sleep(5000);

    }
    public void sleep(int t) {
        try {
            Thread.sleep(t); // Wait for 1 millisecond
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore interrupted status
            // Optionally, log or handle the interruption
        }
    }
    public void updateAll(Pivot pivot, Extension extension, Wrist wrist) {
        while (opModeInInit() || opModeIsActive())
        {


            //pivot.update();

            //extension.update();
            //wrist.update();

            telemetry.addData("pos", pivot.getCurrent());
            telemetry.addData("target", pivot.getTarget());
            telemetry.addData("power", pivot.getPower());
            telemetry.addData("error", pivot.getError());
            telemetry.addData("vel", pivot.getVelocity());
            telemetry.update();
        }
    }

    public void setBucket(Action trajectory , Pivot pivot, Extension extension, Wrist wrist, Claw claw) {
        pivot.setPos("Basket");
        wrist.setPos("Auton Idle");
        //pivot.setkP("Extended");
        sleep(1000);
        extension.setPos("Basket");

        Actions.runBlocking(trajectory);
        wrist.setPos("Basket");
        sleep(500);
        ////claw.directSet(Claw.open);
        sleep(500);
        wrist.setPos("Auton Idle");
        sleep(500);

    }

    public void getBlock(Action trajectory , Pivot pivot, Extension extension, Wrist wrist, Claw claw) {
        extension.setPos("Idle");


        Actions.runBlocking(trajectory);
        pivot.setkP("Normal");
        pivot.setPos("Down");

        wrist.setPos("Intake");
        sleep(1500);
        ////claw.directSet(Claw.closed);
        sleep(500);
       // wrist.setPos("Auton Idle");
        //pivot.setPos("Basket");

    }

}
