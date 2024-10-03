package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeArm {
    public Servo servoIntakeArm;

    public Telemetry telemetry;

    public IntakeArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        servoIntakeArm = hardwareMap.get(Servo.class,HardwareConstant.ServoIntakeArm);

        initIntakeArm();
    }

    public void initIntakeArm() {
        servoIntakeArm.setPosition(0.00);

    }
}
