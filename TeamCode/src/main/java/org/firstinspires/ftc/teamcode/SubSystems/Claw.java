package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    private static final float COLOR_SENSOR_GAIN = 2.0f;
    private Servo clawServo;
    private NormalizedColorSensor colorSensor;
    private Headlights lights;

    private static final double CLAW_OPEN_POSITION = 0.15;
    private static final double CLAW_CLOSE_POSITION = 0.28;

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
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, HardwareConstant.ClawColorSensor);
        colorSensor.setGain(COLOR_SENSOR_GAIN);
        lights = new Headlights(opMode);

        clawServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setPosition(CLAW_CLOSE_POSITION);

        clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
    }

    // creates two states in which the claw opens and closes
    public enum CLAW_SERVO_STATE {
        CLAW_OPEN,
        CLAW_CLOSE,
    }

    public CLAW_SERVO_STATE clawServoState;

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

    public void setAllianceColor(String color) {
        allianceColor = color;
    }

    public String getAllianceColor() {
        return allianceColor;
    }

    // creates two states in which the claw moves up and down


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

