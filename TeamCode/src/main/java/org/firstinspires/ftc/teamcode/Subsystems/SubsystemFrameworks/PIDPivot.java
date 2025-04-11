package org.firstinspires.ftc.teamcode.Subsystems.SubsystemFrameworks;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

public class PIDPivot extends DualMotorPivot{

    private int targetPosition, currentPosition;
    private double outputPower;

    private double Kp, Ki, Kd;
    private PIDController controller;

    public PIDPivot(HardwareMap hwMap, String leftMotorName, String rightMotorName) {
        super(hwMap, leftMotorName, rightMotorName);
    }

    @Override
    public void update() {
        currentPosition = super.leftMotor.getCurrentPosition();
        outputPower = controller.calculate(currentPosition, targetPosition);
        setMotorPower(outputPower);
    }

    public void setMotorPower(double power) {
        super.leftMotor.setPower(power);
        super.rightMotor.setPower(power);
    }

    public void initCoefficients(int Kp, int Ki, int Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void initController() {
        controller = new PIDController(Kp, Ki, Kd);
    }

    public void setCoefficients(int Kp, int Ki, int Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        controller.setPID(Kp, Ki, Kd);
    }
}
