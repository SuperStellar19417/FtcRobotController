package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;

@Autonomous(name = "Preload Deliver + Park", group = "01-Test")
public class SimplePathTestAuto extends LinearOpMode {

    private GamepadController gamepadController;
    private DriveTrain driveTrain;

    // We can transfer this from last autonomous op mode if needed,
    // but most the time we don't need to.
    private final Pose2d startPose = new Pose2d(0, 0,  Math.toRadians(0));

    private Claw claw;
    private Arm arm;
    private IntakeSlide slides;
    private Climber climber;

    private enum STARTING_SIDE {
        RED_RIGHT,
        RED_LEFT,
        BLUE_RIGHT,
        BLUE_LEFT
    }

    public STARTING_SIDE startingSide = STARTING_SIDE.BLUE_LEFT;
    private Action action = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    };
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


        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
     /*   Trajectory test = driveTrain.trajectoryBuilder(startPose)
                .lineToX(10)
                .build();


        Trajectory traj1 = driveTrain.actionBuilder(startPose)
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2); */



        // Create a simple path here
        // We are using RoadRunner's TrajectoryBuilder to create a simple path with a 0,0,0 start pose
        TrajectoryActionBuilder tab1 = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(11,-4));

        TrajectoryActionBuilder tab2 = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(-15, -57))
                .strafeTo(new Vector2d(-25, -57));

        // Create an action that will be run
        Action toBucket = tab1.build();
        Action toAscent = tab2.build();

        // Run the action (s)
        // You can run multiple actions to execute a complex auto. For example :

         /*   Actions.runBlocking(
            new SequentialAction(
                    trajectoryActionChosen,
                    lift.liftUp(),
                    claw.openClaw(),
                    lift.liftDown(),
                    trajectoryActionCloseOut
            )); */

        // TrajectoryActionBuilder creates the path you want to follow and actions are subsystem actions
        // that should be executed once that path is completed.
        Actions.runBlocking(new SequentialAction(toBucket));
        safeWaitSeconds(1000);
        arm.moveArmHighBucketPosition();
        climber.moveClimberUp();
        safeWaitSeconds(1000);
        slides.moveSlideHigh();
        safeWaitSeconds(4000);
        claw.intakeClawOpen();
        safeWaitSeconds(2000);
        slides.moveSlideLow();
        safeWaitSeconds(5200);
        Actions.runBlocking(new SequentialAction(toAscent));

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
            telemetry.addData("DriveTrain Initialized with Pose:", driveTrain.toStringPose2d(driveTrain.pose));
            telemetry.update();
            claw = new Claw(this);
            arm = new Arm(this);
            slides = new IntakeSlide(this);
            climber = new Climber(this);
            gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this, claw, arm, slides, climber);
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
        public void outputTelemetry() {

            telemetry.setAutoClear(true);
            telemetry.addLine("Running Normal TeleOp Mode");

            // Output telemetry messages for subsystems here
            driveTrain.outputTelemetry();

            telemetry.update();
        }

        public Action safeWaitSeconds ( double time){
            ElapsedTime timer = new ElapsedTime(MILLISECONDS);
            timer.reset();
            while (!isStopRequested() && timer.time() < time) {
                //don't even worry about it
            }
            return action;
        }


    public void selectStartingPosition() {

        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addLine("Initializing Superstellar Autonomous Mode:");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:", "");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");
            telemetry.addData("start pose: ", startPose);
            telemetry.addData("current pose: ", driveTrain.pose);
            if (gamepad1.x) {
                startingSide = STARTING_SIDE.BLUE_LEFT;
                break;
            }
            if (gamepad1.y) {
                startingSide = STARTING_SIDE.BLUE_RIGHT;
                break;
            }
            if (gamepad1.b) {
                startingSide = STARTING_SIDE.RED_LEFT;
                break;
            }
            if (gamepad1.a) {
                startingSide = STARTING_SIDE.RED_RIGHT;
                break;
            }

            telemetry.update();
        }
        telemetry.clearAll();
    }
}
