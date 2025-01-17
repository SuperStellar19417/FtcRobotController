package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    //private static final float COLOR_SENSOR_GAIN = 2.0f;
    private Servo clawServo;

    private Servo wristServo;
    //private NormalizedColorSensor colorSensor;
    public Headlights lights;

    //public DistanceSensor distanceSensor;

    //public double distanceFromSubmersible = 0;
    private static final double CLAW_OPEN_POSITION = 0.28;
    private static final double CLAW_CLOSE_POSITION = 0.01;

    private static final double WRIST_MIN_POSITION = 1;
    private static final double WRIST_MAX_POSITION = 0.3;
    private static final double WRIST_DELTA = 0.01;

    private static final double WRIST_UP_POSITION = WRIST_MIN_POSITION;
    private static final double WRIST_MID_POSITION = .6;
    private static final double WRIST_DOWN_POSITION = WRIST_MAX_POSITION;

    public static double WRIST_CURRENT_POSITION = WRIST_UP_POSITION;

    private String allianceColor = "RED";

    public enum DETECTED_COLOR {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    // private DETECTED_COLOR detectedColor = DETECTED_COLOR.UNKNOWN;

    private Action action = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    };

    public Claw(OpMode opMode) {
        clawServo = opMode.hardwareMap.get(Servo.class, HardwareConstant.ClawServo); // 4 control hub
        wristServo = opMode.hardwareMap.get(Servo.class, HardwareConstant.WristServo);
        // colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, HardwareConstant.ClawColorSensor);
        // colorSensor.setGain(COLOR_SENSOR_GAIN);
        lights = new Headlights(opMode);
        //distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, HardwareConstant.DistanceSensor );

        clawServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setPosition(CLAW_CLOSE_POSITION);
        wristServo.setDirection(Servo.Direction.REVERSE);

        clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
        wristServoState = WRIST_SERVO_STATE.WRIST_UP;
    }

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

    public void UpdateColorSensor() {
        // If the claw is closed, we will not detect colors because the claw is covering the sensor
        // and we will always get blue

        if (clawServoState == CLAW_SERVO_STATE.CLAW_CLOSE) {
            //detectedColor = DETECTED_COLOR.UNKNOWN;
            return;
        }

        // Get the normalized colors from the sensor
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//
//        // Based on the ratio (or you can use the raw values) of the colors, determine the detected color
//        if (colors.red > colors.blue && colors.red > colors.green) {
//            detectedColor = DETECTED_COLOR.RED;
//        } else if (colors.blue > colors.red && colors.blue > colors.green) {
//            detectedColor = DETECTED_COLOR.BLUE;
//        } else {
//            detectedColor = DETECTED_COLOR.YELLOW;
//        }
    }



    // public DETECTED_COLOR getDetectedColor() {
//        return detectedColor;
//    }

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
}

