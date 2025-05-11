package org.firstinspires.ftc.teamcode.subsystems.SubsystemFrameworks;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SimpleMotorController {

    private int targetPosition;
    private DcMotorEx motor;

    public SimpleMotorController(HardwareMap hwMap, String motorName) {
        motor = hwMap.get(DcMotorEx.class, motorName);
    }

    public void update() {
        motor.setTargetPosition(targetPosition);
    }
    
    public void setDirection(DcMotorEx.Direction direction) {
        motor.setDirection(direction); 
    }
    
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zpb) {
        motor.setZeroPowerBehavior(zpb);
    }

    public void setMode(DcMotorEx.RunMode mode) {
        motor.setMode(mode);
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
