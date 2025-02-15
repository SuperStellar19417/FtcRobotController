package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class SampleColorLight {
    private NormalizedColorSensor colorSensor;
    private Servo colorLedLight;
    LinearOpMode opMode;

    public SampleColorLight(LinearOpMode opMode) {
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, HardwareConstant.ClawColorSensor);
        colorLedLight = opMode.hardwareMap.get(Servo.class, HardwareConstant.SampleColorLight);
        this.opMode = opMode;
    }


    public enum DETECTED_COLOR {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    private DETECTED_COLOR detectedColor = DETECTED_COLOR.UNKNOWN;

    public DETECTED_COLOR getDetectedColor() {
        return detectedColor;
    }

    public  DETECTED_COLOR detectColor() {
        detectedColor = DETECTED_COLOR.UNKNOWN;

        // get the normalized colors from the sensor
        NormalizedRGBA colors= colorSensor.getNormalizedColors();
//
//        // Based on the ratio (or you can use the raw values) of the colors, determine the detected color
        if (colors.red > colors.blue && colors.red > colors.green) {
            detectedColor = DETECTED_COLOR.RED;
        } else if (colors.blue > colors.red && colors.blue > colors.green) {
            detectedColor = DETECTED_COLOR.BLUE;
        } else {
            detectedColor = DETECTED_COLOR.YELLOW;
        }
        return detectedColor;
    }

    public void runSampleColorDetection(){
        DETECTED_COLOR currentColor = detectColor();

        if (currentColor == DETECTED_COLOR.RED){
            colorLedLight.setPosition(0.3);
        }
        if (currentColor == DETECTED_COLOR.BLUE){
            colorLedLight.setPosition(0.6);
        }
        if (currentColor == DETECTED_COLOR.YELLOW){
            colorLedLight.setPosition(0.35);
        }

    }
    public double getDistance(){
        double distance = 0;
        if (colorSensor instanceof DistanceSensor) {
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        }
        return distance;
    }
}
