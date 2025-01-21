package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Flag {
    private Servo flagServo;
    private OpMode  opMode;

    public enum FLAG_SERVO_STATE {
        FLAG_UP,
        FLAG_DOWN,
    }

    private static final double FLAG_POSITION_UP = 0.05;
    private static final double FLAG_POSITION_DOWN = .55;
    private FLAG_SERVO_STATE servoState;

    public Flag(OpMode opMode) {
        opMode = opMode;
        flagServo = opMode.hardwareMap.get(Servo.class, HardwareConstant.FlagServo);
        flagServo.setDirection(Servo.Direction.FORWARD);
        servoState = FLAG_SERVO_STATE.FLAG_DOWN;
    }

    public void setFlagUp() {
        flagServo.setPosition(FLAG_POSITION_UP);
        servoState = FLAG_SERVO_STATE.FLAG_UP;
    }

    public void setFlagDown() {
        flagServo.setPosition(FLAG_POSITION_DOWN);
        servoState = FLAG_SERVO_STATE.FLAG_DOWN;
    }

    public void toggleFlag() {
        if ( servoState == FLAG_SERVO_STATE.FLAG_DOWN) {
            setFlagUp();
        }
        else {
            setFlagDown();
        }
    }
}
