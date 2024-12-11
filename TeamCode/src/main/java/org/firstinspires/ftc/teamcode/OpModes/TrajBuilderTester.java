package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

@Autonomous (name = "Scoring Auto 1st Ascent TEST", group = "01-Test")
public class TrajBuilderTester extends LinearOpMode {
    private GamepadController gamepadController;
    private DriveTrain driveTrain;

    // We can transfer this from last autonomous op mode if needed,
    // but most the time we don't need to.
    private final Pose2d startPose = new Pose2d(-59.7, 11.16,  Math.toRadians(90));

    private Claw claw;
    private Arm arm;
    private Climber climber;
    @Override
    public void runOpMode() {
        // See https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
        // for more information on how to create a path

        // Initialization code here
        initSubsystems();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        // If Stop is pressed, exit OpMode
        if (isStopRequested()) return;

        // Create a simple path here
        // We are using RoadRunner's TrajectoryBuilder to create a simple path with a 0,0,0 start pose
        TrajectoryActionBuilder toBasket = driveTrain.actionBuilder(startPose)
                .splineTo(new Vector2d(-50.58, 52.38), Math.toRadians(135));

        TrajectoryActionBuilder toSub = driveTrain.actionBuilder(new Pose2d(new Vector2d(-50.58,52.38), Math.toRadians(135)))
                .splineTo(new Vector2d(-9.9, 33.66), Math.toRadians(90))
                .splineTo(new Vector2d(-10.62, 20.7), Math.toRadians(90));


        // Create an action that will be run
        Action basketAction = toBasket.build();
        Action submersibleAction = toSub.build();

        // Run the action (s)
        // You can run multiple actions to execute a complex auto. For example :
        /*
            Actions.runBlocking(
            new SequentialAction(
                    trajectoryActionChosen,
                    lift.liftUp(),
                    claw.openClaw(),
                    lift.liftDown(),
                    trajectoryActionCloseOut
            ));
        */
        // TrajectoryActionBuilder creates the path you want to follow and actions are subsystem actions
        // that should be executed once that path is completed.
        Actions.runBlocking(new SequentialAction(basketAction));
        safeWaitSeconds(2);
        arm.moveArmHighBucketPosition();
        safeWaitSeconds(2);
        claw.intakeClawOpen();
        safeWaitSeconds(1.5);
        arm.moveArmIntakePosition();
        Actions.runBlocking(new SequentialAction(submersibleAction));
        safeWaitSeconds(2);
        climber.moveClimberUp();

    }

    private void initSubsystems() {
        // Initialize all subsystems here
        telemetry.setAutoClear(false);

        // Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        // Initialize drive train
        driveTrain = new DriveTrain(hardwareMap, startPose, this);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this, claw, arm, null, null);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        // Set the bulk mode to auto for control and expansion hubs
        // This optimizes the communication between the robot controller and the expansion hubs and
        // motors, sensors, etc. connected to them.
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addLine("Robot Init Completed");
        telemetry.addLine("====================");
        telemetry.update();
    }

    /**
     * Output telemetry messages to the driver station
     */
    public void outputTelemetry(){

        telemetry.setAutoClear(true);
        telemetry.addLine("Running Normal TeleOp Mode");

        // Output telemetry messages for subsystems here
        driveTrain.outputTelemetry();

        telemetry.update();
    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //don't even worry about it
        }
    }



}
