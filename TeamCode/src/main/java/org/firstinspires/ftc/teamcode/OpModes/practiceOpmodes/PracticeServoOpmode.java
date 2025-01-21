package org.firstinspires.ftc.teamcode.OpModes.practiceOpmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;
@TeleOp(name= "Practice Servo", group= "Tests")

public class PracticeServoOpmode extends LinearOpMode {
    private Servo servo;
    private GamepadController gamepad;


    private Action action = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    };
    /*
     *
     * add two positions + initial position + functions to get them to initial and both positions
     * runOpmode function that allows the user to use the gamepad to access all these positions
     * telemetry updates for each position
     */
    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "testServo");
        servo.setDirection(Servo.Direction.FORWARD);
        gamepad = new GamepadController(gamepad1, null, null, this, null, null, null, null, null);
        if (isStopRequested()) {
            return;
        }

        while (!isStopRequested()) {

            if (gamepad.gp1GetB()) {
                telemetry.addLine("b");
                this.telemetry.update();
                servoPos1();
            } else if (gamepad.gp1GetX()) {
                telemetry.addLine("x");
                this.telemetry.update();
                servoPos2();
            } else if (gamepad.gp1GetA()) {
                telemetry.addLine("a");
                this.telemetry.update();
                servoInit();
            }


        }
    }
        public enum SERVO_STATE {
            SERVO_POS1,
            SERVO_POS2,
            SERVO_INIT,
        }

        public SERVO_STATE servoState;

        private void servoPos1 () {
            servo.setPosition(0.8);
            servoState = SERVO_STATE.SERVO_POS1;
        }

        private void servoPos2 () {
            servo.setPosition(0.3);
            servoState = SERVO_STATE.SERVO_POS2;
        }

        private void servoInit () {
            servo.setPosition(0.0);
            servoState = SERVO_STATE.SERVO_INIT;
        }



}
