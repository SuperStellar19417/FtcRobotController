package org.firstinspires.ftc.teamcode.SubSystems;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class SampleColorLight {
    public enum DETECTED_COLOR {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    private NormalizedColorSensor colorSensor;
    private Servo colorLedLight;
    private DETECTED_COLOR detectedColor = DETECTED_COLOR.UNKNOWN;
    private LinearOpMode opMode;
    private float[] hsvValues = {0F, 0F, 0F};
    private float gain = 2.0f;
    private static final double DISTANCE_THRESHOLD = 2.5;
    private NormalizedRGBA colors = new NormalizedRGBA();
    private double distance = 0;

    public SampleColorLight(LinearOpMode opMode) {
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, HardwareConstant.ClawColorSensor);
        colorLedLight = opMode.hardwareMap.get(Servo.class, HardwareConstant.SampleColorLight);

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(gain);

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        this.opMode = opMode;
    }

    public DETECTED_COLOR getDetectedColor() {
        return detectedColor;
    }

    public void runSampleColorDetection() {
        // Detect the color and set the LED light accordingly

        if (colorSensor instanceof DistanceSensor) {
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        }

        // get the normalized colors from the sensor
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Use HSV values to determine the color
        // For example, to get the hue value (which is a value from 0 to 360), you can do the following:
        float hue = hsvValues[0];
        // The hue value is a value from 0 to 360. The red color is at the lower end (0) and at the higher end (360). It is a circle/
        // We can use this to determine the color detected by the sensor.
        // For example, if the hue value is between 0 and 60, we can say that the detected color is red.
        // If the hue value is between 210 and 270, we can say that the detected color is blue.
        // If the hue value is between 30 and 90, we can say that the detected color is yellow.
        // These values are just approximations and you can adjust them based on your sensor and lighting conditions.

        detectedColor = DETECTED_COLOR.UNKNOWN;

        // Only set color if we are close to a sample (DISTANCE_THRESHOLD cm)
        if (distance <= DISTANCE_THRESHOLD) {
            if (hue < 60) {
                detectedColor = DETECTED_COLOR.RED;
            } else if (hue < 210) {
                detectedColor = DETECTED_COLOR.YELLOW;
            } else if (hue < 270) {
                detectedColor = DETECTED_COLOR.BLUE;
            }
        }
        // IF DETECTED COLOR IS NOT UNKNOWN AND CLAW IS CLOSED AND THE DISTANCE IS LESS THAN LIKE IDK 20.


////        // Based on the ratio (or you can use the raw values) of the colors, determine the detected color
//        if (colors.red > colors.blue && colors.red > colors.green) {
//            detectedColor = DETECTED_COLOR.RED;
//        } else if (colors.blue > colors.red && colors.blue > colors.green) {
//            detectedColor = DETECTED_COLOR.BLUE;
//        } else {
//            detectedColor = DETECTED_COLOR.YELLOW;
//        }

        setLightColor();

    }

    private void setLightColor() {
        if (detectedColor == DETECTED_COLOR.RED){
            colorLedLight.setPosition(0.3);
        }
        if (detectedColor == DETECTED_COLOR.BLUE){
            colorLedLight.setPosition(0.6);
        }
        if (detectedColor == DETECTED_COLOR.YELLOW){
            colorLedLight.setPosition(0.35);
        }
        if (detectedColor == DETECTED_COLOR.UNKNOWN){
            colorLedLight.setPosition(0.0);
        }
    }

    public double getDistance(){
        return distance;
    }

    public void outputTelemetry() {
        opMode.telemetry.addData("Detected Color", detectedColor);
        opMode.telemetry.addData("Distance (cm) claw sensor", distance);
       // opMode.telemetry.addData("RGB", "%.2f : %.2f : %.2f", colors.red, colors.green, colors.blue);
       // opMode.telemetry.addData("HSV", "%.2f : %.2f : %.2f", hsvValues[0],  hsvValues[1] , hsvValues[2]);
    }
}
