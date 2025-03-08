package org.firstinspires.ftc.teamcode.OpModes.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

/**
 * This class is used to test the climber and arm subsystem based climbing
 * GP2 DPAD UP - move arm slightly up
 * GP2 DPAD DOWN - move arm slightly down
 * GP2 LEFT BUMPER - move climber slightly up
 * GP2 LEFT TRIGGER - move climber slightly down
 */
@TeleOp(name = "Climber Test", group = "00-Teleop")
@Disabled
public class ClimberTest extends LinearOpMode {

    private GamepadController gamepad;
    private Arm arm;
    private Climber climber;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(this);
        climber = new Climber(this);

        // climber = new Climber(this);
        gamepad = new GamepadController(gamepad1, gamepad2, null, this, null, arm, null, null, null, null);

        waitForStart();
        while(!isStopRequested()) {
            telemetry.update();
            while(opModeIsActive()) {
                // Arm tests
                if (gamepad.gp2GetDpad_upPress()) {
                    arm.moveArmSlightlyUp();
                    telemetry.addLine(arm.getCurrentArmPosition() + " ");
                } else if (gamepad.gp2GetDpad_downPress()) {
                    arm.moveArmSlightlyDown();
                    telemetry.addLine(arm.getCurrentArmPosition() + " ");
                }

                if(gamepad.gp2GetButtonAPress()) {
                    arm.moveArmLowBasketPosition();
                } else if(gamepad.gp2GetButtonBPress()) {
                //    arm.moveArmHighBucketPosition();
                } else if(gamepad.gp2GetButtonXPress()) {
                    arm.moveArmLowRungPosition();
                } else if(gamepad.gp2GetButtonYPress()) {
                    arm.moveArmHighRungPosition(false);
                }

                if(gamepad.gp2GetButtonAPress()) {
                    arm.moveArmLowBasketPosition();
                } else if(gamepad.gp2GetButtonBPress()) {
          //          arm.moveArmHighBucketPosition();
                } else if(gamepad.gp2GetButtonXPress()) {
                    arm.moveArmLowRungPosition();
                } else if(gamepad.gp2GetButtonYPress()) {
                    arm.moveArmHighRungPosition(false);
                }

                // Climber tests
                if (gamepad.gp2GetLeftBumperPress()) {
                    climber.moveClimberSlightlyUp();
                    telemetry.addLine(climber.getClimberTargetPosition() + " ");
                } else if (gamepad.gp2GetLeftTriggerPress()) {
                    climber.moveClimberSlightlyDown(true);
                    telemetry.addLine(climber.getClimberTargetPosition() + " ");
                }
            }
        }
    }
}
