package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private static final float COLOR_SENSOR_GAIN = 2.0f;
    private Servo clawServo;

    private Servo wristServo;
    private NormalizedColorSensor colorSensor;
    private Headlights lights;

    private static final double CLAW_POSITION_OPEN = 0.15;
    private static final double CLAW_POSITION_CLOSE = 0.35;

    private static final double WRIST_POSITION_UP = 0.8;
    private static final double WRIST_POSITION_DOWN = 0.2;
    private static final double WRIST_POSITION_STRAIGHT = 0.0;

    private String allianceColor = "RED";

    public enum DETECTED_COLOR {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    private DETECTED_COLOR detectedColor = DETECTED_COLOR.UNKNOWN;

    private Action action = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    };

    public Claw(OpMode opMode) {
        clawServo = opMode.hardwareMap.get(Servo.class, HardwareConstant.ClawServo); // 4 control hub
        wristServo = opMode.hardwareMap.get(Servo.class, HardwareConstant.WristServo);
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, HardwareConstant.ClawColorSensor);
        colorSensor.setGain(COLOR_SENSOR_GAIN);
        lights = new Headlights(opMode);

        clawServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);

        clawServo.setPosition(CLAW_POSITION_CLOSE);
        clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;

        wristServo.setPosition(WRIST_POSITION_UP);
        wristServoState = WRIST_SERVO_STATE.WRIST_UP;
    }

    // creates two states in which the claw opens and closes
    public enum CLAW_SERVO_STATE {
        CLAW_OPEN,
        CLAW_CLOSE,
    }
    public enum WRIST_SERVO_STATE {
        WRIST_UP,
        WRIST_DOWN,
        WRIST_STRAIGHT
    }

    public CLAW_SERVO_STATE clawServoState;
    public WRIST_SERVO_STATE wristServoState;

    public void UpdateColorSensor() {
        // If the claw is closed, we will not detect colors because the claw is covering the sensor
        // and we will always get blue

        if (clawServoState == CLAW_SERVO_STATE.CLAW_CLOSE) {
            detectedColor = DETECTED_COLOR.UNKNOWN;
            return;
        }

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Based on the ratio (or you can use the raw values) of the colors, determine the detected color
        if (colors.red > colors.blue && colors.red > colors.green) {
            detectedColor = DETECTED_COLOR.RED;
        } else if (colors.blue > colors.red && colors.blue > colors.green) {
            detectedColor = DETECTED_COLOR.BLUE;
        } else {
            detectedColor = DETECTED_COLOR.YELLOW;
        }
    }

    public DETECTED_COLOR getDetectedColor() {
        return detectedColor;
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
        wristServo.setPosition(WRIST_POSITION_UP);
        wristServoState = WRIST_SERVO_STATE.WRIST_UP;
    }
    public void wristDown() {
        wristServo.setPosition(WRIST_POSITION_DOWN);
        wristServoState = WRIST_SERVO_STATE.WRIST_DOWN;
    }

    public Action intakeClawOpen() {
        clawServo.setPosition(CLAW_POSITION_OPEN);
        clawServoState = CLAW_SERVO_STATE.CLAW_OPEN;
        lights.headlightOff();
        return action;
    }

    public Action intakeClawClose() {
        clawServo.setPosition(CLAW_POSITION_CLOSE);
        clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
        lights.headlightOn();
        return action;
    }
}

