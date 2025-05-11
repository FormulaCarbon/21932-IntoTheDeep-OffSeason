package org.firstinspires.ftc.teamcode.Subsystems.SubsystemFrameworks;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {

    private DcMotorEx frontLeft, backLeft, frontRight, backRight;

    private double x, y, rx, d;
    private double curveFactor = 1;
    private double[] multipliers = {1, 1, 1, 1};

    public MecanumDrive(HardwareMap hwMap, String[] motorNames) {
        frontLeft = hwMap.get(DcMotorEx.class, motorNames[0]);
        backLeft = hwMap.get(DcMotorEx.class, motorNames[1]);
        frontRight = hwMap.get(DcMotorEx.class, motorNames[2]);
        backRight = hwMap.get(DcMotorEx.class, motorNames[3]);
    }

    public void reverseLeftMotors() {

    }

}