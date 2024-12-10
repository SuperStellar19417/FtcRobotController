package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

@TeleOp(name = "Practice DC Motor", group = "00-Teleop")
public class PracticeDcMotorOpmode extends LinearOpMode {

    GamepadController gamepad;
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
        gamepad= new GamepadController(gamepad1, null, null,null,null,null,null,null);
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

    private boolean gp1GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp1ButtonYLast && gamepad1.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = gamepad1.y;
        return isPressedButtonY;
    }

    private boolean gp1GetButtonXPress() {
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


    private void runMotorToPosition(int position) {

        telemetry.addData("Target Position", position);
        practiceMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        practiceMotor.setTargetPosition(position);
        practiceMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        practiceMotor.setPower(0.5);
    }

}
