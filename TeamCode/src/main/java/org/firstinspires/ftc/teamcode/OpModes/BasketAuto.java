package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Flag;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(name = "Auto - Basket", group = "01-Auto", preselectTeleOp = "Normal TeleOp")
public class BasketAuto extends LinearOpMode {
    private GamepadController gamepadController;
    private MecanumDrive driveTrain;
    private Claw claw;
    private Arm arm;
    private IntakeSlide intakeSlide;
    private Flag flag;


    private final Pose2d startPose = new Pose2d(0, 0,  Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization code here
        initSubsystems();

        // Build Autonomous trajectory to be used based on starting position selected
        // actionBuilder builds from the drive steps passed to it
        // Close the claw
        claw.intakeClawClose();

        telemetry.addLine("Waiting for Start to be pressed");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        // If Stop is pressed, exit OpMode
        if (isStopRequested()) return;

        // Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            // Perform the auto steps

            telemetry.addLine("Running actions");
            telemetry.update();

            //telemetry.setAutoClear(true);

            Actions.runBlocking(
                new SequentialAction(
                        driveToBucketAction()
                    ));

            arm.moveArmLowBasketPosition();
            intakeSlide.moveSlideHigh();
            claw.wristMid();
            safeWaitSeconds(1);

            Actions.runBlocking(
                    new SequentialAction(
                            moveForwardABitAction(),
                            turnAction(55)
                    ));

            claw.intakeClawOpen();
            safeWaitSeconds(2);

            Actions.runBlocking(
                    new SequentialAction(
                            moveBackABitAction(),
                            turnAction(-35),
                            moveToAscentLocation(),
                            moveBackwardsToAscent()
                    ));

            flag.setFlagUp();
            intakeSlide.moveSlideLow();
            safeWaitSeconds(1);
            arm.moveArmIntakePosition();
            arm.stopMotors();
            safeWaitSeconds(10); // let the op mode expire so flag stays up

/*
            intakeSlide.moveSlideLow();
            arm.moveArmToAutoSamplePickupPosition();
            safeWaitSeconds(3);
            claw.intakeClawClose();
            safeWaitSeconds(2);
            arm.moveArmLowBasketPosition();

            // Go the bucket
            Actions.runBlocking(
                    new SequentialAction(
                            turnAction(128),
                            moveToBucketAction()
                    )
            );


            intakeSlide.moveSlideHigh();
            claw.wristMid();
            claw.intakeClawOpen();

            safeWaitSeconds(1);

            Actions.runBlocking(
                    new SequentialAction(
                            moveBackABitAction(),
                            turnAction(-128),
                            moveToFirstSampleAction()
                    ));

            intakeSlide.moveSlideLow();
            arm.moveArmIntakePosition();
            */


            /*Actions.runBlocking(
                    new SequentialAction(
                            moveToChamberAction()
                    )
                    */





            telemetry.addLine("Done");
            telemetry.update();
        }
    }

    private void initSubsystems() {
        // Initialize all subsystems here
        telemetry.setAutoClear(false);

        // Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        // Initialize drive train
        driveTrain = new MecanumDrive(hardwareMap, startPose);
        telemetry.addData("DriveTrain Initialized with Pose:", Utils.toStringPose2d(startPose));
        telemetry.update();
        arm = new Arm(this);
        claw = new Claw(this);
        intakeSlide = new IntakeSlide(this);
        flag = new Flag(this);

        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this, claw, arm, intakeSlide, null, flag);
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

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //don't even worry about it
        }
    }

    private Action driveToBucketAction() {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(5) // 4 inches forward
                .turn(Math.toRadians(90))
                .lineToY(31);

        return tab.build();
    }

    private Action turnAction(double degrees){
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .turn(Math.toRadians(degrees));

        return tab.build();
    }

    private Action moveForwardABitAction() {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(7) ;

        return tab.build();
    }

    private Action moveBackABitAction() {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(-24) ;

        return tab.build();
    }

    private Action moveToFirstSampleAction() {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(10) ;

        return tab.build();
    }

    private Action moveToBucketAction() {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(12) ;

        return tab.build();
    }

    private Action moveToAscentLocation() {
        Vector2d ascentPos = new Vector2d(0,-29);
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .strafeToConstantHeading(ascentPos);

        return tab.build();
    }
    
    private Action moveBackwardsToAscent() {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(-14) ;

        return tab.build();
    }
}
