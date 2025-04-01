package org.firstinspires.ftc.teamcode.Subsystems.SubsystemFrameworks;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DualMotorPivot {
    private int targetPosition;
    private DcMotorEx leftMotor, rightMotor;

    public DualMotorPivot(HardwareMap hwMap, String leftMotorName, String rightMotorName) {
        leftMotor = hwMap.get(DcMotorEx.class, leftMotorName);
        rightMotor = hwMap.get(DcMotorEx.class, rightMotorName);
    }

    public void update() {
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);
    }

    public void setLeftMotorPower(double power) {
        leftMotor.setPower(power);
    }

    public void setRightMotorPower(double power) {
        rightMotor.setPower(power);
    }
    
    public void setLeftMotorDirection(DcMotorEx.Direction direction) {
        leftMotor.setDirection(direction);
    }

    public void setRightMotorDirection(DcMotorEx.Direction direction) {
        rightMotor.setDirection(direction);
    }
    
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zpb) {
        leftMotor.setZeroPowerBehavior(zpb);
        rightMotor.setZeroPowerBehavior(zpb);
    }

    public void setMode(DcMotorEx.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getLeftMotorCurrentPosition() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightMotorCurrentPosition() {
        return rightMotor.getCurrentPosition();
    }
}
