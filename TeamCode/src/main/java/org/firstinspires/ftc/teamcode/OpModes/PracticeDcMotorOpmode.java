package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PracticeDcMotorOpmode extends LinearOpMode {
    DcMotorEx practiceMotor;
    private final double MAX_VELOCITY = 3120;
    private final int RUNNING_VELOCITY = 2000;

    boolean gp1ButtonBLast = false;
    boolean gp1ButtonXLast = false;
    boolean gp1ButtonYLast = false;

    /*
     * add two positions + initial position + functions to get them to initial and both positions
     * runOpmode function that allows the user to use the gamepad to access all these positions
     * telemetry updates for each position
     */
    @Override
    public void runOpMode() throws InterruptedException {
        practiceMotor = hardwareMap.get(DcMotorEx.class, "practiceMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        telemetry.addLine("Start Pressed");
        telemetry.update();

        int Position1 = 1355;
        telemetry.addData("Position 1", Position1);

        int Position2 = 1725;
        telemetry.addData("Position 2", Position2);

        while (opModeIsActive()) {

            if (isStopRequested())
                break;


            if (gp1GetButtonBPress()) {
                telemetry.addLine("Gamepad 1 B Pressed");
                runMotorToPosition(Position1);
            }
            else if (gp1GetButtonXPress()) {
                telemetry.addLine("Gamepad 1 X Pressed");
                runMotorToPosition(Position2);
            }
            else if (gp1GetButtonYPress()){
                telemetry.addLine("Gamepad 1 Y Pressed");
                runMotorToPosition(0);
            }
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private void runMotorToPosition(int position) {

        telemetry.addData("Target Position", position);
        practiceMotor.setTargetPosition(position);
        practiceMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        practiceMotor.setVelocity(RUNNING_VELOCITY);
    }

    public boolean gp1GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp1ButtonYLast && gamepad1.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = gamepad1.y;
        return isPressedButtonY;
    }

    public boolean gp1GetButtonXPress() {
        boolean isPressedButtonX = false;
        if (!gp1ButtonXLast && gamepad1.a) {
            isPressedButtonX = true;
        }
        gp1ButtonXLast = gamepad1.a;
        return isPressedButtonX;
    }

    public boolean gp1GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp1ButtonBLast && gamepad1.a) {
            isPressedButtonB = true;
        }
        gp1ButtonBLast = gamepad1.a;
        return isPressedButtonB;
    }
}
