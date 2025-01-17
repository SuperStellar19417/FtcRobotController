package org.firstinspires.ftc.teamcode.OpModes.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

@TeleOp(name = "Claw Test", group = "00-Teleop")
@Disabled
public class ClawTest extends LinearOpMode {

    GamepadController gamepad;
    Claw claw;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new GamepadController(gamepad1, gamepad2, null, this, claw, null, null, null);
        claw = new Claw(this);

        waitForStart();



        if(isStopRequested()) {
            return;
        }

        while(!isStopRequested()) {

            if(gamepad.gp1GetButtonBPress()) {
                this.telemetry.update();
                claw.setAllianceColor("RED");
            } else if (gamepad.gp1GetButtonXPress()) {
                telemetry.update();
                claw.setAllianceColor("BLUE");
            }

            telemetry.addData("claw state", claw.getClawServoState());
            telemetry.addData("claw alliance color", claw.getAllianceColor());
            telemetry.update();

            if(gamepad.gp1GetRightBumperPress()) {
                this.telemetry.update();
                claw.intakeClawOpen();
            } else if (gamepad.gp1GetLeftBumperPress()) {
                telemetry.update();
                claw.intakeClawClose();
            }
        }
    }
}
