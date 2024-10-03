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
    public enum INTAKE_CLAW_SERVO_STATE{
        INTAKE_CLAW_OPEN,

        INTAKE_CLAW_CLOSE,
    }

    public INTAKE_CLAW_SERVO_STATE intakeClawServoState = INTAKE_CLAW_SERVO_STATE.INTAKE_CLAW_OPEN;

    public enum INTAKE_CLAW_WRIST_STATE {
        INTAKE_WRIST_DOWN,

        INTAKE_WRIST_UP,
    }

    public INTAKE_CLAW_WRIST_STATE intakeClawWristState = INTAKE_CLAW_WRIST_STATE.INTAKE_WRIST_DOWN;



    public void intakeClawOpen(){
        rightIntakeServo.setPosition(0.00);
        leftIntakeServo.setPosition(0.00);
        intakeClawServoState = INTAKE_CLAW_SERVO_STATE.INTAKE_CLAW_OPEN;
    }

    public void intakeClawClose(){
        rightIntakeServo.setPosition(0.00);
        leftIntakeServo.setPosition(0.00);
        intakeClawServoState = INTAKE_CLAW_SERVO_STATE.INTAKE_CLAW_CLOSE;
    }

    public void intakeClawUp(){
        wristIntakeServo.setPosition(0.00);
        intakeClawWristState = INTAKE_CLAW_WRIST_STATE.INTAKE_WRIST_UP;
    }
    public void intakeClawDown() {
        wristIntakeServo.setPosition(0.00);
        intakeClawWristState = INTAKE_CLAW_WRIST_STATE.INTAKE_WRIST_DOWN;
    }
}

