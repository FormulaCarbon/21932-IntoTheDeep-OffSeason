package org.firstinspires.ftc.teamcode.auton;
import java.lang.Math;
//import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

//@Config
@Autonomous(name = "Arush Sample", group = "Sensor")
public class ArushAuton extends LinearOpMode {

    public static double inx = 0;
    public static double iny = 0;

    public void runOpMode() throws InterruptedException {
        //Initial position set
        Rotation2d inr = new Rotation2d(Math.PI);
        Pose2d initialized = new Pose2d(inx, iny, inr);





    }


}
