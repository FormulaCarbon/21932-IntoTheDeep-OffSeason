package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

@Config
public class ActiveIntake {

    private CRServo spin;
    private Servo clamp;
    ColorRangefinder sensor;
    NormalizedRGBA colors, lastColors;
    public static int ledBrightness = 100;
    public static double clampUp = 0.26, clampDown = 0.56, spitTime = 0.5;

    private double tarPower, tarPos = clampUp;

    public static double waitTime = 0.0, redThresh = 0.007, blueThresh = 0.005, greenThresh = 0.01, redYellowThresh = 0.01, distanceThresh = 60, minThresh = 0.002, redGreenThresh = 0.002;

    private double redDelta = 0, blueDelta = 0, greenDelta = 0, distance = 100, minDelta, redGreenDelta;
    public static double staticRed = 0.026, staticBlue = 0.039, staticGreen = 0.049, holdPow = 0, outPow = 0.35, outPow2 = -0.35;

    private String blockColor = "None";

    private boolean holdingBlock = false;
    public static boolean auto = false;

    ElapsedTime timer = new ElapsedTime(), spitTimer = new ElapsedTime();

    public ActiveIntake(HardwareMap hwMap, HashMap<String, String> config) {
        spin = hwMap.crservo.get(config.get("spinner"));
        clamp = hwMap.servo.get(config.get("clamp"));
        sensor = new ColorRangefinder(hwMap.get(RevColorSensorV3.class, config.get("colorSensor")));
        sensor.setLedBrightness(ledBrightness);
        colors = sensor.emulator.getNormalizedColors();
        distance = getDistance();
    }

    public void update() {
        spin.setPower(tarPower);
        clamp.setPosition(tarPos);

        if (timer.seconds() > waitTime) {
            lastColors = colors;
            colors = sensor.emulator.getNormalizedColors();
            redDelta = colors.red - staticRed;
            blueDelta = colors.blue - staticBlue;
            greenDelta = colors.green - staticGreen;
            distance = getDistance();

            holdingBlock = distance < distanceThresh;
            minDelta = Math.max(redDelta, Math.max(greenDelta, blueDelta));
            redGreenDelta = Math.abs(redDelta - greenDelta);

/*
            if (redDelta > redYellowThresh && greenDelta > greenThresh && holdingBlock) {
                blockColor = "Yellow";
            } else if (redDelta > redThresh && holdingBlock) {
                blockColor = "Red";
            } else if (blueDelta > blueThresh && holdingBlock) {
                blockColor = "Blue";
            } else {
                blockColor = "None";
            }*/
            if (minDelta > minThresh && holdingBlock)
            {
                if (redGreenDelta < redGreenThresh && blueDelta != minDelta) {
                    blockColor = "Yellow";
                }
                else if (redDelta == minDelta) {
                    blockColor = "Red";
                }
                else if (blueDelta == minDelta) {
                    blockColor = "Blue";
                }
                else if (greenDelta == minDelta) {
                    blockColor = "Yellow";
                }
                else {
                    blockColor = "None";
                }
            }
            else {
                blockColor = "None";
            }
            timer.reset();
        }
    }

    public void clamp() {
        tarPos = clampDown;
    }

    public void unclamp() {
        tarPos = clampUp;
    }

    public void intake() {
        tarPower = -1.0;
    }

    public void outtake() {
        tarPower = outPow;
    }

    public void outtake2() {
        tarPower = outPow2;
        //unclamp();
    }



    public void off() {
        tarPower = 0;
    }

    public void hold() {
        tarPower = holdPow;
    }

    public NormalizedRGBA getColors() {
        return colors;
    }

    public NormalizedRGBA getLastColors() {
        return colors;
    }

    public double getDistance() {
        return sensor.readDistance();
    }

    public String getBlockColor() {
        return blockColor;
    }

    public double getDelta(String color) {
        if (color.equals("Red"))
        {
            return redDelta;
        }
        else if (color.equals("Blue")) {
            return blueDelta;
        }
        else {
            return greenDelta;
        }
    }

    public double time()
    {
        return timer.seconds();
    }

    public void spit(double time) {
        if (time < spitTime) {
            tarPower = -1.0;
        }
        else {
            intake();
        }
    }

    public void auto(String avoid) {
        if (auto) {
            if (blockColor.equals("None") || blockColor.equals(avoid)) {
                intake();
            } else {
                off();
                clamp();
            }

            if (blockColor.equals(avoid)) {
                unclamp();
            }
        }
    }

    public void setAuto(boolean n) {
        auto = n;
    }

    public boolean isHoldingBlock() {
        return holdingBlock;
    }

    public void simpleUpdate() {
        spin.setPower(tarPower);
        clamp.setPosition(tarPos);
    }
}
