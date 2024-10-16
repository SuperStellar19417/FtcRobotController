package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    public Servo clawServo;
    public Servo wristServo;
    public NormalizedColorSensor colorSensor;
    public String allianceColor = "RED";

    public Telemetry telemetry;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        clawServo = hardwareMap.get(Servo.class, HardwareConstant.ClawServo); // 4 control hub
     //   wristServo = hardwareMap.get(Servo.class, HardwareConstant.WristServo);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, HardwareConstant.ClawColorSensor);

        initIntakeClaw();
    }

    // creates two states in which the claw opens and closes
    public enum CLAW_SERVO_STATE{
        CLAW_OPEN,

        CLAW_CLOSE,
    }

    public CLAW_SERVO_STATE clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;

    // creates two states in which the claw moves up and down
    public enum CLAW_WRIST_STATE {
        WRIST_DOWN,
        WRIST_UP
    }

    public CLAW_WRIST_STATE clawWristState = CLAW_WRIST_STATE.WRIST_DOWN;
    // Sa
    public void initIntakeClaw() {
        clawServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setPosition(0.00);
        clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
        clawWristState = CLAW_WRIST_STATE.WRIST_DOWN;
    }
    // Starting positions of the servos for the opened claw
    public void intakeClawOpen(){
        clawServo.setPosition(0.5);
        clawServoState = CLAW_SERVO_STATE.CLAW_OPEN;
    }

    // Starting positions of the servos for the closed claw

    public void intakeClawClose() {
        if(allianceColor.equals("BLUE")) {
            if (colorSensor.getNormalizedColors().blue > 0.002){
                clawServo.setPosition(0.00);
            //    leftIntakeServo.setPosition(0.00);
                clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
            }
        } else {
            if(colorSensor.getNormalizedColors().red > 0.002){
                clawServo.setPosition(0.00);
            //    leftIntakeServo.setPosition(0.00);
                clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
            }
        }
    }

    //TODO: move wrist somewhere else

    // Starting positions of the servos for a wrist that is up
    public void intakeClawUp(){
        wristServo.setPosition(0.7);
        clawWristState = CLAW_WRIST_STATE.WRIST_UP;
    }

    // Starting positions of the servos for a wrist that is down
    public void intakeClawDown() {
        wristServo.setPosition(0.3);
        clawWristState = CLAW_WRIST_STATE.WRIST_DOWN;
    }
}

