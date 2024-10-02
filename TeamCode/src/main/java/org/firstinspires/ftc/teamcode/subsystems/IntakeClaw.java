package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeClaw {
    public Servo rightIntakeServo;
    public Servo leftIntakeServo;
    public Servo wristIntakeServo;

    public Telemetry telemetry;

    public IntakeClaw(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");

        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");

        wristIntakeServo = hardwareMap.get(Servo.class, "wristIntakeServo");
    }
    public void intakeClawOpen(){

    }
}

