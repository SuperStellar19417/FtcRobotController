package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;
@TeleOp(name= "Practice Servo", group= "00-Teleop")
public class PracticeServoOpmode extends LinearOpMode {
    private Servo Servo;
    private GamepadController Gamepad;


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
    public void runOpMode() throws InterruptedException {
        Servo = hardwareMap.get(Servo.class, "testServo");
        Gamepad = new GamepadController(gamepad1, null, null);
        if (isStopRequested()) {
            return;
        }

        while (!isStopRequested()) {

            if (gamepad.gp1GetButtonBPress()) {
                this.telemetry.update();
                servoPos1();
            } else if (gamepad.gp1GetButtonXPress()) {
                this.telemetry.update();
                servoPos2();
            } else if (gamepad.gp1GetButtonAPress()) {
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
            Servo.setPosition(0.8);
            servoState = SERVO_STATE.SERVO_POS1;
        }

        private void servoPos2 () {
            Servo.setPosition(0.3);
            servoState = SERVO_STATE.SERVO_POS2;
        }

        private void servoInit () {
            Servo.setPosition(0.0);
            servoState = SERVO_STATE.SERVO_INIT;
        }





}
