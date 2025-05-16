package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Config
public class PTO {
    private Servo pto;

    public static double on = 0.59, off = 0.78;

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
