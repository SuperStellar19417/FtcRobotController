package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;

@TeleOp(name = "Normal TeleOp", group = "00-Teleop")
public class NormalTeleOp extends LinearOpMode {

    private GamepadController gamepadController;
    // Declare subsystems here
    private DriveTrain driveTrain;

    private enum ALLIANCE {
        BLUE,
        RED
    }

    private ALLIANCE allianceSelection = ALLIANCE.RED;

    // We can tranfer this from last autonoumous opmode if needed,
    // but most the time we don't need to.
    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    private Claw claw;
    private Arm arm;
    private IntakeSlide slide;
    private Climber climber;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization code here
        initSubsystems();

       /* if(gamepadController.gp1GetX()) {
            claw.allianceColor = "BLUE";
        } else if (gamepadController.gp1GetX()) {
            claw.allianceColor = "RED";
        }*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addLine("X for BLUE ALLIANCE : O for RED ALLIANCE");
        if (gamepadController.gp1GetX()) {
            allianceSelection = ALLIANCE.BLUE;
        } else if (gamepadController.gp1GetX()) {
            allianceSelection = ALLIANCE.RED;
        }

        telemetry.addLine("Start Pressed");
        telemetry.update();

        // If Stop is pressed, exit OpMode
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {
            outputTelemetry();
            telemetry.update();

            while (opModeIsActive()) {
                // TeleOp code here
                gamepadController.runSubSystems();
                outputTelemetry();
            }
        }
    }

    private void initSubsystems() throws InterruptedException {
        // Initialize all subsystems here

        telemetry.setAutoClear(false);

        // Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        // Intialize drive train
        driveTrain = new DriveTrain(hardwareMap, startPose, this);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;

        telemetry.addData("DriveTrain Initialized with Pose:", driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        // claw = new Claw(hardwareMap, telemetry);
        arm = new Arm(this);
        climber = new Climber(this);
        slide = new IntakeSlide(this);
        claw = new Claw(this);
        if (allianceSelection == ALLIANCE.RED) {
            //   claw.allianceColor = "RED";
        } else {
            //claw.allianceColor = "BLUE";
        }
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this, claw, arm, slide, climber);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();


        // Set the bulk mode to auto for control and expansion hubs
        // This optimizes the communication between the robot controller and the expansion hubs and
        // motors, sensors, etc. connected to them.
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addLine("Robot Init Completed ");
        telemetry.addLine("====================");
        telemetry.update();
    }


    /**
     * Output telemetry messages to the driver station
     */
    public void outputTelemetry() {

        telemetry.setAutoClear(true);
        telemetry.addLine("Running Normal TeleOpMode");

        // Output telemetry messages for susbsystems here
        driveTrain.outputTelemetry();
        telemetry.addData("Climber Target Position: ", climber.getClimberTargetPosition());
        telemetry.addData("Climber Motor Position: ", climber.getClimberMotorPosition());
        telemetry.addData("Claw state: ", claw.getClawServoState());
        telemetry.addData("Wrist state: ", claw.getWristServoState());
        telemetry.addData("Slides Target Position: ", slide.getTargetPosition());
        telemetry.addData("Slides Motor Position: ", slide.getMotorPosition());
        telemetry.addData("Arm Motor Position: ", arm.getCurrentArmPosition());
        telemetry.addData("Arm Motor Encoder: ", arm.getCurrentArmEncoderValue());
        telemetry.addData("joystick position", gamepadController.gp2GetLeftStickY());

        telemetry.update();
    }

}
