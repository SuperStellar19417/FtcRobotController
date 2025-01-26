package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

// we need to review these steps and then fill in the movement coordinates
// Our auto steps for 1 specimen on run and park are:
// 0. Start with claw closed and wrist up with specimen loaded
// 1. Move forward a bit (to get out of the way of other robot)
// 2. Strafe left
// 3. Raise arm and slides
// 4. Move forward to almost touch the submersible
// 5. Wrist mid down
// 6. Pull slides back
// 7. Wait for 2 seconds
// 8. Open Claw
// 9. Wait for 2 seconds
// 10. Lower arm and slides
// 11. Move back to almost touching the wall
// 12. Strafe right to observation zone (parking)
// 13.Set zero power to arm

@Autonomous(name = "Auto - Specimen", group = "01-Auto", preselectTeleOp = "Normal TeleOp")
public class SpecimenAuto extends LinearOpMode {
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

        // Close the claw and move wrist up
        claw.intakeClawClose();
        claw.wristUp();

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

            // Get close to submersible
            arm.moveArmHighRungPosition();
            intakeSlide.moveSlideHigh();
            intakeSlide.retractSlide(false);

            Actions.runBlocking(
                new SequentialAction(
                        driveToSubmersibleAction()
                    ));

            // Raise arms and slides

            claw.wristMid();
            safeWaitSeconds(1);

            // Move closer to submersible
            Actions.runBlocking(
                    new SequentialAction(
                            moveForwardABitAction()
                    ));

            // Move wrist mid point
            claw.wristMid();

            // Pull slides back
            intakeSlide.moveSlideLow();

            // Wait a little bit and open the claw
            safeWaitSeconds(1);
            Actions.runBlocking(
                    new SequentialAction(
                            moveBackABitAction()
                    ));
            claw.intakeClawOpen();
            safeWaitSeconds(2);

            // Lets go park
            Actions.runBlocking(
                    new SequentialAction(
                            moveToParkPostion(),
                            moveBackParkAction()
                    )
            );
            claw.wristUp();
            safeWaitSeconds(1);


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

    /**
     * Move forward a bit and then strafe left
     * @return
     */
    private Action driveToSubmersibleAction() {
        //Vector2d headingVector = new Vector2d(0,8);

        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(21); // 10 inches forward

        return tab.build();
    }

    /**
     * Move back and then strafe right to park position
     * @return
     */
    private Action moveToParkPostion() {
        Vector2d headingVector = new Vector2d(0,-44);

        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(-19) //
                .strafeToConstantHeading(headingVector)
                .lineToX(4);

        return tab.build();
    }

    private Action turnAction(double degrees){
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .turn(Math.toRadians(degrees));

        return tab.build();
    }

    private Action moveForwardABitAction() {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(3) ;

        return tab.build();
    }



    private Action moveBackABitAction() {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(-5) ;

        return tab.build();
    }

    private Action moveBackParkAction() {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .lineToX(-10) ;

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

  /*  private Action moveToChamberAction(){
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .turn(Math.toRadians(90))
        return tab.build();
    } */
}
