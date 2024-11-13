package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

@TeleOp(name = "Motor Test", group = "00-Teleop")
public class MotorTest extends LinearOpMode {

    private GamepadController gamepad;
    private Arm arm;
    private Climber climber;

    @Override
    public void runOpMode() throws InterruptedException {
        //arm = new Arm(this);
        climber = new Climber(this);
        gamepad = new GamepadController(gamepad1, gamepad2, null, this, null, arm, null, null);

        waitForStart();
        while(!isStopRequested()) {
            telemetry.update();
            while(opModeIsActive()) {
                if (gamepad.gp2GetDpad_upPress()) {
                    climber.moveClimberSlightlyUp();
                    telemetry.addLine(arm.armPositionCount + " ");
                } else if (gamepad.gp2GetDpad_downPress()) {
                    climber.moveClimberSlightlyDown();
                    telemetry.addLine(arm.armPositionCount + " ");
                }

              /*  if(gamepad.gp2GetButtonAPress()) {
                    arm.moveArmLowBucketPosition();
                } else if(gamepad.gp2GetButtonBPress()) {
                    arm.moveArmHighBucketPosition();
                } else if(gamepad.gp2GetButtonXPress()) {
                    arm.moveArmLowRungPosition();
                } else if(gamepad.gp2GetButtonYPress()) {
                    arm.moveArmHighRungPosition();
                }

                if(gamepad.gp2GetButtonAPress()) {
                    arm.moveArmLowBucketPosition();
                } else if(gamepad.gp2GetButtonBPress()) {
                    arm.moveArmHighBucketPosition();
                } else if(gamepad.gp2GetButtonXPress()) {
                    arm.moveArmLowRungPosition();
                } else if(gamepad.gp2GetButtonYPress()) {
                    arm.moveArmHighRungPosition();
                } */
            }
        }
    }
}
