package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.HardwareConstant;


public class ColorLights {
    private NormalizedColorSensor colorSensor;
    private Servo hlTop;

    public ColorLights (LinearOpMode opMode) {
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, HardwareConstant.ClawColorSensor);
        hlTop = opMode.hardwareMap.get(Servo.class, HardwareConstant.HeadlightTop);
    }


    public enum DETECTED_COLOR {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    private DETECTED_COLOR detectedColor = DETECTED_COLOR.UNKNOWN;

    public  DETECTED_COLOR getColor() {
        // If the claw is closed, we will not detect colors because the claw is covering the sensor
        // and we will always get blue
        // G get the normalized colors from the sensor
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

    public void setHlTopColor(){
        DETECTED_COLOR currentColor = getColor();
        if (currentColor == DETECTED_COLOR.RED){
            hlTop.setPosition(0.3);
        }
        if (currentColor == DETECTED_COLOR.BLUE){
            hlTop.setPosition(0.6);
        }
        if (currentColor == DETECTED_COLOR.YELLOW){
            hlTop.setPosition(0.35);
        }

    }
    public double getDistance(){
        double distance = 0;
        if (colorSensor instanceof DistanceSensor) {
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            opMode.telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            opMode.telemetry.update();
        }
        return distance;
    }



}
