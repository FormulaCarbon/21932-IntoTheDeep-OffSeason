package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class PTO {
    private Servo pto;

    private static double on = 0, off = 0.1;

    private double targetPos = off;
    public PTO(HardwareMap hardwareMap, HashMap<String, String> deviceConf) {
        pto = hardwareMap.servo.get(deviceConf.get("pto"));
    }

    public void update()
    {
        pto.setPosition(targetPos);
    }

    public void activate() {
        targetPos = on;
    }

    public void deactivate() {
        targetPos = off;
    }
}
