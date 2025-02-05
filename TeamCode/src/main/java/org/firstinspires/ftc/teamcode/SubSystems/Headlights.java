package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;

public class Headlights {
    public Servo hlLeft;
    public Servo hlRight;
    public Servo hlTop;

    public Headlights(OpMode opmode) {
        hlLeft = opmode.hardwareMap.get(Servo.class, HardwareConstant.HeadlightLeft);
        hlRight = opmode.hardwareMap.get(Servo.class, HardwareConstant.HeadlightRight);
        hlTop = opmode.hardwareMap.get(Servo.class, HardwareConstant.HeadlightTop);
        hlRight.setPosition(0);
        hlLeft.setPosition(0);
        hlTop.setPosition(0);
    }

    public void headlightOn(){
        hlLeft.setPosition(0.5);
        hlRight.setPosition(0.5);
    }

    public void headlightOff(){
        hlLeft.setPosition(0);
        hlRight.setPosition(0);
    }

    public void topHeadLightOn(double detectedColor){
        hlTop.setPosition(detectedColor);
    }

    public void topHeadLightOff(){
        hlTop.setPosition(0);
    }
}