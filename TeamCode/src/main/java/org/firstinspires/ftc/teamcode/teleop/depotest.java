package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Util;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.SpecMec;

@TeleOp
@Config
public class depotest extends LinearOpMode {
    public static double change = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
        Util util = new Util();
        Wrist wrist = new Wrist(hardwareMap, util.deviceConf);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.dpad_right)
            {
                wrist.setPos("Intake");
            }
            if (gamepad1.dpad_left)
            {
                wrist.setPos("Basket");
            }
            if (gamepad1.dpad_up)
            {
                wrist.setPos("Idle");
            }
            if (gamepad1.dpad_down)
            {
                wrist.setPos("Start");
            }

            wrist.update();

            telemetry.update();


        }
    }
}