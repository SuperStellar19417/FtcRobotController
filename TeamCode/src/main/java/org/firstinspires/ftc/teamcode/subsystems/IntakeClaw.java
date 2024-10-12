package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeClaw {
    public Servo rightIntakeServo;
    public Servo leftIntakeServo;
    public Servo wristIntakeServo;
    public NormalizedColorSensor colorSensor;
    public String allianceColor;

    public Telemetry telemetry;

    public IntakeClaw(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        rightIntakeServo = hardwareMap.get(Servo.class, HardwareConstant.RightIntakeServo);

        leftIntakeServo = hardwareMap.get(Servo.class, HardwareConstant.LeftIntakeServo);

        wristIntakeServo = hardwareMap.get(Servo.class, HardwareConstant.WristIntakeServo);

        initIntakeClaw();    }

    // creates two states in which the claw opens and closes
    public enum INTAKE_CLAW_SERVO_STATE{
        INTAKE_CLAW_OPEN,

        INTAKE_CLAW_CLOSE,
    }

    public INTAKE_CLAW_SERVO_STATE intakeClawServoState = INTAKE_CLAW_SERVO_STATE.INTAKE_CLAW_CLOSE;

    // creates two states in which the claw moves up and down
    public enum INTAKE_CLAW_WRIST_STATE {
        INTAKE_WRIST_DOWN,

        INTAKE_WRIST_UP,
    }

    public INTAKE_CLAW_WRIST_STATE intakeClawWristState = INTAKE_CLAW_WRIST_STATE.INTAKE_WRIST_DOWN;
    // Sa
    public void initIntakeClaw() {
        rightIntakeServo.setPosition(0.00);
        leftIntakeServo.setPosition(0.00);
        intakeClawServoState = INTAKE_CLAW_SERVO_STATE.INTAKE_CLAW_CLOSE;

        wristIntakeServo.setPosition(0.00);
        intakeClawWristState = INTAKE_CLAW_WRIST_STATE.INTAKE_WRIST_DOWN;
    }
    // Starting positions of the servos for the opened claw
    public void intakeClawOpen(){
        rightIntakeServo.setPosition(0.5);
        leftIntakeServo.setPosition(0.5);
        intakeClawServoState = INTAKE_CLAW_SERVO_STATE.INTAKE_CLAW_OPEN;
    }

    // Starting positions of the servos for the closed claw

    public void intakeClawClose() {
        if(allianceColor.equals("BLUE")) {
            if (colorSensor.getNormalizedColors().blue > 0.5){
                rightIntakeServo.setPosition(0.00);
                leftIntakeServo.setPosition(0.00);
                intakeClawServoState = INTAKE_CLAW_SERVO_STATE.INTAKE_CLAW_CLOSE;
            }
        } else {
            if(colorSensor.getNormalizedColors().red > 0.5){
                rightIntakeServo.setPosition(0.00);
                leftIntakeServo.setPosition(0.00);
                intakeClawServoState = INTAKE_CLAW_SERVO_STATE.INTAKE_CLAW_CLOSE;
            }
        }

    }

    //TODO: move wrist somewhere else

    // Starting positions of the servos for a wrist that is up
    public void intakeClawUp(){
        wristIntakeServo.setPosition(0.7);
        intakeClawWristState = INTAKE_CLAW_WRIST_STATE.INTAKE_WRIST_UP;
    }

    // Starting positions of the servos for a wrist that is down
    public void intakeClawDown() {
        wristIntakeServo.setPosition(0.3);
        intakeClawWristState = INTAKE_CLAW_WRIST_STATE.INTAKE_WRIST_DOWN;
    }
}

