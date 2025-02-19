package org.firstinspires.ftc.teamcode.OpModes.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystems.HardwareConstant;

/**
 * A test servo opmode to move a servo with the dpad up and down buttons.
 * This is used to test the range of motion of a servo.
 */
@TeleOp(name = "Servo Range Test", group = "Tests")
public class ServoRangeTest extends LinearOpMode {
    private double MIN_POSITION = 0.0;
    private double MAX_POSITION = 1.0;

    private Servo servo;
    private double servoPosition = MIN_POSITION;
    private double servoDelta = 0.01;

    private String servoName = HardwareConstant.ClawServo;
    private boolean reverseDirection = false;

    private boolean gp1ButtonYLast = false;
    private boolean gp1ButtonALast = false;

    private boolean gp1ButtonXLast = false;
    private boolean gp1ButtonBLast = false;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, servoName);

        if (reverseDirection) {
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }

        servo.setPosition(MIN_POSITION);
        telemetry.addLine("Sevo Position 0");
        telemetry.update();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Press Y to move up");
            telemetry.addLine("Press A to move down");
            telemetry.addLine("Press X to set min position");
            telemetry.addLine("Press B to set max position");

            if(gp1GetButtonYPress()) {
                servoPosition += servoDelta;
                if (servoPosition > MAX_POSITION) {
                    servoPosition = MAX_POSITION;
                }
            } else if(gp1GetButtonAPress()) {
                servoPosition -= servoDelta;
                if (servoPosition < MIN_POSITION) {
                    servoPosition = MIN_POSITION;
                }
            } else if (gp1GetXButtonPress()) {
                servoPosition = MIN_POSITION;
            } else if (gp1GetButtonBPress()) {
                servoPosition = MAX_POSITION;
            }

            servo.setPosition(servoPosition);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }

    private boolean gp1GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp1ButtonYLast && gamepad1.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = gamepad1.y;
        return isPressedButtonY;
    }

    private boolean gp1GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp1ButtonALast && gamepad1.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = gamepad1.a;
        return isPressedButtonA;
    }

    private boolean gp1GetXButtonPress() {
        boolean isPressedButtonX = false;
        if (!gp1ButtonXLast && gamepad1.x) {
            isPressedButtonX = true;
        }
        gp1ButtonXLast = gamepad1.x;
        return isPressedButtonX;
    }

    private boolean gp1GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp1ButtonBLast && gamepad1.b) {
            isPressedButtonB = true;
        }
        gp1ButtonBLast = gamepad1.b;
        return isPressedButtonB;
    }
}
