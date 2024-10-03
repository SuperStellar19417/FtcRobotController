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
        rightIntakeServo = hardwareMap.get(Servo.class, HardwareConstant.RightIntakeServo);

        leftIntakeServo = hardwareMap.get(Servo.class, HardwareConstant.LeftIntakeServo);

        wristIntakeServo = hardwareMap.get(Servo.class, HardwareConstant.WristIntakeServo);
    }
    public void intakeClawOpen(){

    }
}

