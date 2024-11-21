package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    public Servo clawServo;
    public NormalizedColorSensor colorSensor;
    public String allianceColor = "RED";

    public Telemetry telemetry;
    public Headlights lights;

    private Action action = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    };

    public Claw(OpMode opMode) {
        clawServo = opMode.hardwareMap.get(Servo.class, HardwareConstant.ClawServo); // 4 control hub
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, HardwareConstant.ClawColorSensor);
        lights = new Headlights(opMode);

        clawServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setPosition(0.3);

        clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
    }

    // creates two states in which the claw opens and closes
    public enum CLAW_SERVO_STATE {
        CLAW_OPEN,

        CLAW_CLOSE,
    }

    public CLAW_SERVO_STATE clawServoState;

    // creates two states in which the claw moves up and down


     // Starting positions of the servos for the opened claw
    public Action intakeClawOpen() {
        clawServo.setPosition(0.1);
        clawServoState = CLAW_SERVO_STATE.CLAW_OPEN;
        lights.headlightOff();
        return action;
    }


    // Starting positions of the servos for the closed claw

    public Action intakeClawClose() {
       // if (colorSensor.getNormalizedColors().red > 0.001 && colorSensor.getNormalizedColors().green > 0.001) {
            clawServo.setPosition(0.3);
            //    leftIntakeServo.setPosition(0.00);
            clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
            lights.headlightOn();
            return action;
     //   } else {


          /*  if (allianceColor.equals("BLUE")  ) {

                if (colorSensor.getNormalizedColors().blue > 0.0017 && colorSensor.getNormalizedColors().blue > colorSensor.getNormalizedColors().red) {
                    clawServo.setPosition(0.00);
                    //    leftIntakeServo.setPosition(0.00);
                    clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;
                }
            } else {

                if (colorSensor.getNormalizedColors().red > 0.0013 && colorSensor.getNormalizedColors().red > colorSensor.getNormalizedColors().blue) {
                    clawServo.setPosition(0.00);
                    //    leftIntakeServo.setPosition(0.00);
                    clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;

                } else {
                    if (colorSensor.getNormalizedColors().red > 0.0023 && colorSensor.getNormalizedColors().blue > 0.0021 && colorSensor.getNormalizedColors().green > 0.0032) {
                        clawServo.setPosition(0.00);
                        clawServoState = CLAW_SERVO_STATE.CLAW_CLOSE;


                    }

                }
                */

            }
         // }


            //TODO: move wrist somewhere else

            // Starting positions of the servos for a wrist that is up




        // Starting positions of the servos for a wrist that is down


    }

