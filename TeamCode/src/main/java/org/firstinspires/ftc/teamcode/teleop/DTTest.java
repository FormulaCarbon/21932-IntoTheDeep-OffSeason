package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Util;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.SpecMec;

import java.util.HashMap;

@TeleOp
@Config
public class DTTest extends LinearOpMode {

    public static HashMap<String, String> deviceConf = new HashMap<String, String>();

    @Override
    public void runOpMode() throws InterruptedException {
        deviceConf.put("frontLeft",       "frontLeftMotor");
        deviceConf.put("backLeft",        "backLeftMotor");
        deviceConf.put("frontRight",      "frontRightMotor");
        deviceConf.put("backRight",       "backRightMotor");

        Drive drive = new Drive(hardwareMap, deviceConf);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            drive.getXYZ(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

            drive.update();

        }

    }
}
