package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Claw {
    //private static final float COLOR_SENSOR_GAIN = 2.0f;
    private Servo clawServo;
    private Servo wristServo;
    public Headlights lights;

    public DistanceSensor distanceSensor;


    //
    public double distanceFromSubmersible = 0;
    private static final double CLAW_OPEN_POSITION = 0.45;
    private static final double CLAW_CLOSE_POSITION = 0.18;

    private static final double WRIST_MIN_POSITION = 0;
    private static final double WRIST_MAX_POSITION = 0.59;
    private static final double WRIST_DELTA = 0.01;

    private static final double WRIST_UP_POSITION = WRIST_MIN_POSITION;
    private static final double WRIST_MID_POSITION = .6; // Position for pickup
    private static final double WRIST_DOWN_POSITION = WRIST_MAX_POSITION;

    public static double WRIST_CURRENT_POSITION = WRIST_UP_POSITION;

    private String allianceColor = "RED";

    // creates two states in which the claw opens and closes
    public enum CLAW_SERVO_STATE {
        CLAW_OPEN,
        CLAW_CLOSE,
    }
    public enum WRIST_SERVO_STATE {
        WRIST_UP,
        WRIST_MID,
        WRIST_DOWN,
    }

    public CLAW_SERVO_STATE clawServoState;
    public WRIST_SERVO_STATE wristServoState;

    private Action action = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    };

    public Claw(OpMode opMode) {
        clawServo = opMode.hardwareMap.get(Servo.class, HardwareConstant.ClawServo); // 4 control hub
        wristServo = opMode.hardwareMap.get(Servo.class, HardwareConstant.WristServo);

        lights = new Headlights(opMode);
        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, HardwareConstant.DistanceSensor );

       // clawServo.setDirection(Servo.Direction.FORWARD);
      //  clawServo.setPosition(CLAW_CLOSE_POSITION);
       // wristServo.setDirection(Servo.Direction.REVERSE);

        wristServo.setPosition(WRIST_UP_POSITION);

        clawServoState = CLAW_SERVO_STATE.CLAW_OPEN;
        wristServoState = WRIST_SERVO_STATE.WRIST_UP;
    }


    public CLAW_SERVO_STATE getClawServoState() {
        return clawServoState;
    }

    public WRIST_SERVO_STATE getWristServoState(){return wristServoState;}

    public void setAllianceColor(String color) {
        allianceColor = color;
    }

    public String getAllianceColor() {
        return allianceColor;
    }

    // creates two states in which the claw moves up and down
    public void wristUp() {
        WRIST_CURRENT_POSITION = WRIST_UP_POSITION;
        wristServo.setPosition(WRIST_CURRENT_POSITION);
        wristServoState = WRIST_SERVO_STATE.WRIST_UP;
    }
    public void wristDown() {
        WRIST_CURRENT_POSITION = WRIST_DOWN_POSITION;
        wristServo.setPosition(WRIST_CURRENT_POSITION);
        wristServoState = WRIST_SERVO_STATE.WRIST_DOWN;
    }
    public void wristMid() {
        WRIST_CURRENT_POSITION = WRIST_MID_POSITION;
        wristServo.setPosition(WRIST_CURRENT_POSITION);
        wristServoState = WRIST_SERVO_STATE.WRIST_MID;
    }

    public void wristSlightlyUp() {
        WRIST_CURRENT_POSITION+= WRIST_DELTA;
        if (WRIST_CURRENT_POSITION > WRIST_MAX_POSITION) {
            WRIST_CURRENT_POSITION = WRIST_MAX_POSITION;
        }
        wristServo.setPosition(WRIST_CURRENT_POSITION);
    }

    public void wristSlightlyDown() {
        WRIST_CURRENT_POSITION-= WRIST_DELTA;
        if (WRIST_CURRENT_POSITION < WRIST_MIN_POSITION) {
            WRIST_CURRENT_POSITION = WRIST_MIN_POSITION;
        }
        wristServo.setPosition(WRIST_CURRENT_POSITION);
    }


    // Starting positions of the servos for the opened claw
    public Action intakeClawOpen() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
        clawServoState = CLAW_SERVO_STATE.CLAW_OPEN;
        lights.headlightOff();

        return action;
    }


    // Starting positions of the servos for the closed claw

    public Action intakeClawClose() {
        clawServo.setPosition(CLAW_CLOSE_POSITION);
        clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
        lights.headlightOn();
        return action;
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}

