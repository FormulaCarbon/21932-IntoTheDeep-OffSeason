package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Config
public class Wrist {

    private Servo rotation, forearm, bicep;

    private double bicepPos, forearmPos, rotationPos;

    public static HashMap<String, Double> bicepPositions = new HashMap<String, Double>();
    public static HashMap<String, Double> forearmPositions = new HashMap<String, Double>();

    public static double autoPos = 0.63, intakepos = 0, bBasket = 0.88, fBasket = 0.61, bIdle = 0.85, fIdle = 0.1356, bIntake = 0.87, fIntake = 0.0, fDrop = 0.39;
    public static double bStart = 0.74, fStart = 0.85;
    public static double[] rotationPositions = new double[5];

    public Wrist(HardwareMap hwMap, HashMap<String, String> config) {
        bicep = hwMap.servo.get(config.get("bicep"));
        forearm = hwMap.servo.get(config.get("forearm"));
        rotation = hwMap.servo.get(config.get("rotation"));

        bicepPositions.put("Intake",      bIntake);
        bicepPositions.put("Basket",      bBasket);
        bicepPositions.put("Ready",       0.81);
        bicepPositions.put("Idle",        0.805);
        bicepPositions.put("Start",       bStart);
        bicepPositions.put("Auton Idle",  0.805);

        forearmPositions.put("Intake",      fIntake);
        forearmPositions.put("Basket",      fBasket);
        forearmPositions.put("Ready",       0.0367);
        forearmPositions.put("Idle",        fIdle);
        forearmPositions.put("Start",       fStart);
        forearmPositions.put("Drop",        fDrop);
        forearmPositions.put("Auton Idle",  fIdle);

        rotationPositions[0] = 0.21;
        rotationPositions[1] = 0.35;
        rotationPositions[2] = 0.49;
        rotationPositions[3] = 0.07;
        rotationPositions[4] = autoPos;


    }

    public void update()
    {
        bicep.setPosition(bicepPos);
        forearm.setPosition(forearmPos);
        rotation.setPosition(rotationPos);
    }

    public void setBicepPos(String pos)
    {
        bicepPos = bicepPositions.get(pos);
    }
    public void setForearmPos(String pos)
    {
        forearmPos = forearmPositions.get(pos);
    }
    public void setRotationPos(int pos)
    {
        rotationPos = rotationPositions[pos];
    }

    public void setPos(String pos) {
        bicepPos = bicepPositions.get(pos);
        forearmPos = forearmPositions.get(pos);
    }

}
