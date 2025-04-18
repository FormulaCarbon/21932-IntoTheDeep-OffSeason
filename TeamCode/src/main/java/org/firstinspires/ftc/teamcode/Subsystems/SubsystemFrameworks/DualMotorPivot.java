package org.firstinspires.ftc.teamcode.Subsystems.SubsystemFrameworks;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DualMotorPivot {
//test

    public SimpleMotorController leftMotor, rightMotor;

    public DualMotorPivot(HardwareMap hwMap, String leftMotorName, String rightMotorName) {
        leftMotor = new SimpleMotorController(hwMap, leftMotorName);
        rightMotor = new SimpleMotorController(hwMap, rightMotorName);
    }

    public void update() {
        leftMotor.update();
        rightMotor.update();
    }

    public void setTargetPosition(int targetPosition) {
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);
    }
}
