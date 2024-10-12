package org.firstinspires.ftc.teamcode.SubSystems;


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
        rightOuttakeClaw = hardwareMap.get(Servo.class, HardwareConstant.RightOuttakeServo);
        leftOuttakeClaw = hardwareMap.get(Servo.class, HardwareConstant.LeftOuttakeServo);
        wristOuttakeClaw = hardwareMap.get(Servo.class, HardwareConstant.WristOuttakeServo);

        initOuttakeClaw();
    }
    public enum OUTTAKE_CLAW_SERVO_STATE{
        OUTTAKE_CLAW_OPEN,

        OUTTAKE_CLAW_CLOSE

    }

    public OUTTAKE_CLAW_SERVO_STATE outtakeClawServoState = OUTTAKE_CLAW_SERVO_STATE.OUTTAKE_CLAW_OPEN;

    public enum OUTTAKE_CLAW_WRIST_STATE {

        OUTTAKE_WRIST_DOWN,

        OUTTAKE_WRIST_UP,

    }

    public OUTTAKE_CLAW_WRIST_STATE outtakeClawWristState = OUTTAKE_CLAW_WRIST_STATE.OUTTAKE_WRIST_DOWN;

    public void initOuttakeClaw() {
        rightOuttakeClaw.setPosition(0.00);;
        leftOuttakeClaw.setPosition(0.00);
        outtakeClawServoState = OUTTAKE_CLAW_SERVO_STATE.OUTTAKE_CLAW_CLOSE;

        wristOuttakeClaw.setPosition(0.00);
        outtakeClawWristState = OUTTAKE_CLAW_WRIST_STATE.OUTTAKE_WRIST_DOWN;
    }

    public void outtakeClawOpen(){
        rightOuttakeClaw.setPosition(0.00);;
        leftOuttakeClaw.setPosition(0.00);
        outtakeClawServoState = OUTTAKE_CLAW_SERVO_STATE.OUTTAKE_CLAW_OPEN;
    }

    public void outtakeClawClose(){
        rightOuttakeClaw.setPosition(0.00);;
        leftOuttakeClaw.setPosition(0.00);
        outtakeClawServoState = OUTTAKE_CLAW_SERVO_STATE.OUTTAKE_CLAW_CLOSE;
    }

    public void outtakeWristUp(){
        wristOuttakeClaw.setPosition(0.00);
        outtakeClawWristState = OUTTAKE_CLAW_WRIST_STATE.OUTTAKE_WRIST_UP;
    }

    public void outtakeWristDown(){
        wristOuttakeClaw.setPosition(0.00);
        outtakeClawWristState = OUTTAKE_CLAW_WRIST_STATE.OUTTAKE_WRIST_DOWN;

    }

}
