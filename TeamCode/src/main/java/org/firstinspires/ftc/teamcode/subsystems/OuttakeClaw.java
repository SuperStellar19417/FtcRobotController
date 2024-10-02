package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeClaw {
    public Servo rightOuttakeClaw;
    public Servo leftOuttakeClaw;
    public Servo wristOuttakeClaw;

    public Telemetry telemetry;

    public OuttakeClaw(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        rightOuttakeClaw = hardwareMap.get(Servo.class, "rightOuttakeClaw");
        leftOuttakeClaw = hardwareMap.get(Servo.class, "leftOuttakeClaw");
        wristOuttakeClaw = hardwareMap.get(Servo.class, "wristOuttakeClaw");


    }

}
