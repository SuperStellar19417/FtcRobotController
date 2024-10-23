package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * GamepadController class is the main interface for the driver to control the robot.
 * We react to button presses and stick movements to control the robot.
 */
public class GamepadController {

    // References to hardware components and subsystems
    private Gamepad gamepad1, gamepad2;
    private DriveTrain driveTrain;
    private LinearOpMode opMode;
    private Claw claw;
    private Arm arm;
    private LinearSlide slide;

    /**
     * Constructor for GamepadController
     * @param gamepad1 - Gamepad 1
     * @param gamepad2 - Gamepad 2
     * @param driveTrain - DriveTrain subsystem
     * @param opMode - LinearOpMode reference
     */
    public GamepadController(Gamepad gamepad1,
                             Gamepad gamepad2,
                             DriveTrain driveTrain,
                             LinearOpMode opMode,
                             Claw claw,
                             Arm arm,
                             LinearSlide slide
    ) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.driveTrain = driveTrain;
        this.opMode = opMode;
        this.claw = claw;
        this.arm = arm;
        this.slide = slide;
    }

    public void runSubSystems(){
        runDriveTrain();
       // runClaw();
        runArm();
        runSlides();

    }

    public void runArm() {
        if(gp1GetButtonAPress()) {
            arm.moveArmLowBucketPosition();
        } else if(gp1GetButtonBPress()) {
            arm.moveArmHighBucketPosition();
        } else if(gp1GetButtonXPress()) {
            arm.moveArmLowRungPosition();
        } else if(gp1GetButtonYPress()) {
            arm.moveArmHighRungPosition();
        }
    }


    /**
     *runByGamepad is the main controller function that runs each subsystem controller based on states
     */

    /**
     * runByGamepadRRDriveModes sets modes for Road Runner such as ROBOT and FIELD Centric Modes. <BR>
     */
    public void runDriveTrain() {

        driveTrain.gamepadInputTurn = gp1TurboMode(-gp1GetRightStickX());

        if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
            driveTrain.gamepadInput = new Vector2d(
                    -gp1TurboMode(gp1GetLeftStickY()),
                    -gp1TurboMode(gp1GetLeftStickX()));
        }

        /* We can add this if we decide to use Field Centric Mode
        if (driveTrain.driveType == DriveTrain.DriveType.FIELD_CENTRIC){

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) { // Red Alliance
                driveTrain.gamepadInput = driveTrain.rotateFieldCentric(
                        -gp1TurboMode(gp1GetLeftStickY()),
                        -gp1TurboMode(gp1GetLeftStickX()),
                        driveTrain.pose.heading.log()
                );
                driveTrain.gamepadInput = driveTrain.pose.heading.inverse().times(
                        new Vector2d(-driveTrain.gamepadInput.x, driveTrain.gamepadInput.y));
            };

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) { // Blue Alliance
//                driveTrain.gamepadInput = driveTrain.rotateFieldCentric(
//                        -gp1TurboMode(gp1GetLeftStickY()),
//                        -gp1TurboMode(gp1GetLeftStickX()),
//                        -driveTrain.pose.heading.log()
//                );

                driveTrain.gamepadInput = driveTrain.pose.heading.plus(Math.PI).inverse().times(
                        new Vector2d(-driveTrain.gamepadInput.x, driveTrain.gamepadInput.y));
            }
        }
        */

        driveTrain.driveNormal();
    }

    //*********** KEY PAD MODIFIERS BELOW ***********

    //**** Gamepad buttons

    //Records last button press to deal with single button presses doing a certain methods
    boolean gp1ButtonALast = false;
    boolean gp1ButtonBLast = false;
    boolean gp1ButtonXLast = false;
    boolean gp1ButtonYLast = false;
    boolean gp1RightBumperLast = false;
    boolean gp1LeftBumperLast = false;
    boolean gp1Dpad_upLast = false;
    boolean gp1Dpad_downLast = false;
    boolean gp1Dpad_leftLast = false;
    boolean gp1Dpad_rightLast = false;
    boolean gp1LeftTriggerLast = false;
    boolean gp1RightTriggerLast = false;

    boolean gp2ButtonALast = false;
    boolean gp2ButtonBLast = false;
    boolean gp2ButtonXLast = false;
    boolean gp2ButtonYLast = false;
    boolean gp2RightBumperLast = false;
    boolean gp2LeftBumperLast = false;
    boolean gp2Dpad_upLast = false;
    boolean gp2Dpad_downLast = false;
    boolean gp2Dpad_leftLast = false;
    boolean gp2Dpad_rightLast = false;
    boolean gp2LeftTriggerLast = false;
    boolean gp2RightTriggerLast = false;



    public void runClaw() {
        if(gp1GetLeftBumper()) {
            if(claw.clawServoState == Claw.CLAW_SERVO_STATE.CLAW_OPEN) {
                claw.intakeClawClose();
            } else {
                claw.intakeClawOpen();
            }
        }
    }

    public void runSlides() {
        if(gp2GetDpad_downPress()) {
            slide.setSlidePositionHold();
        } else if (gp2GetDpad_upPress()) {
            slide.setSlidePositionExtend();
        }
    }


    /**
     * Method to convert linear map from gamepad1 and gamepad2 stick input to a cubic map
     *
     * @param stickInput input value of button stick vector
     * @return Cube of the stick input reduced to 25% speed
     */
    public double limitStick(double stickInput) {
        return (stickInput * stickInput * stickInput * 0.33); //0.25
    }

    /**
     * Method to implement turbo speed mode - from reduced speed of 25% of cubic factor to
     * 100% speed, but controlled by acceleration of the force of pressing the Right Tigger.
     *
     * @param stickInput input value of button stick vector
     * @return modified value of button stick vector
     */
    public double gp1TurboMode(double stickInput) {

        double acceleration_factor;
        double rightTriggerValue;

        double turboFactor;

        rightTriggerValue = gp1GetRightTrigger();
        //acceleration_factor = 1.0 + 3.0 * rightTriggerValue;
        acceleration_factor = 1.0 + 2.0 * rightTriggerValue;
        turboFactor = limitStick(stickInput) * acceleration_factor;
        return turboFactor;
    }
    public double gp2TurboMode(double stickInput) {

        double acceleration_factor;
        double rightTriggerValue;

        double turboFactor;

        rightTriggerValue = gp2GetRightTrigger();
        //acceleration_factor = 1.0 + 3.0 * rightTriggerValue;
        acceleration_factor = 1.0 + 2.0 * rightTriggerValue;
        turboFactor = limitStick(stickInput) * acceleration_factor;
        return turboFactor;
    }

    /**
     * Methods to get the value of gamepad Left stick X for Pan motion X direction.
     * This is the method to apply any directional modifiers to match to the X plane of robot.
     * No modifier needed for Hazmat Freight Frenzy Robot.
     *
     * @return gpGamepad1.left_stick_x
     */
    public double gp1GetLeftStickX() {
        return gamepad1.left_stick_x;
    }
    public double gp2GetLeftStickX() {
        return gamepad2.left_stick_x;
    }

    /**
     * Methods to get the value of gamepad Left stick Y for Pan motion Y direction.
     * This is the method to apply any directional modifiers to match to the Y plane of robot.
     * For Hazmat Freight Frenzy Robot, Y direction needs to be inverted.
     *
     * @return gpGamepad1.left_stick_y
     */
    public double gp1GetLeftStickY() { return gamepad1.left_stick_y; }
    public double gp2GetLeftStickY() { return gamepad2.left_stick_y; }

    /**
     * Methods to get the value of gamepad Right stick X to keep turning.
     * This is the method to apply any directional modifiers to match to the turn direction robot.
     * No modifier needed for Hazmat Freight Frenzy Robot.
     *
     * @return gpGamepad1.right_stick_x
     */
    public double gp1GetRightStickX() {
        return gamepad1.right_stick_x;
    }
    public double gp2GetRightStickX() {
        return gamepad2.right_stick_x;
    }
    public double gp1GetRightStickY() {
        return gamepad1.right_stick_y;
    }
    public double gp2GetRightStickY() {
        return gamepad2.right_stick_y;
    }

    /**
     * Methods to get the value of gamepad Right Trigger for turbo mode (max speed).
     * This is the method to apply any modifiers to match to action of turbo mode for each driver preference.
     * For Hazmat Freight Frenzy Right Trigger pressed means turbo mode on.
     *
     * @return gpGamepad1.right_trigger
     * @return gpGamepad2.right_trigger
     */
    public double gp1GetRightTrigger() {
        return gamepad1.right_trigger;
    }
    public double gp2GetRightTrigger() {
        return gamepad2.right_trigger;
    }

    /**
     * gp1 right trigger press cubic value when pressed
     * @return if right trigger pressed
     */
    public boolean gp1GetRightTriggerPress() {
        boolean isPressedRightTrigger = false;
        if (!gp1RightTriggerLast && (gp1GetRightTrigger()>0.7)) {
            isPressedRightTrigger = true;
        }
        gp1RightTriggerLast = (gp1GetRightTrigger()>0.7);
        return isPressedRightTrigger;
    }

    /**
     * gp2 right trigger press cubic value when pressed
     * @return
     */
    public boolean gp2GetRightTriggerPress() {
        boolean isPressedRightTrigger = false;
        if (!gp2RightTriggerLast && (gp2GetRightTrigger()>0.7)) {
            isPressedRightTrigger = true;
        }
        gp2RightTriggerLast = (gp2GetRightTrigger()>0.7);
        return isPressedRightTrigger;
    }

    /**
     * Methods to get the value of gamepad Left Trigger
     *
     * @return gpGamepad1.left_trigger
     * @return gpGamepad2.left_trigger
     */
    public double gp1GetLeftTrigger() {
        return gamepad1.left_trigger;
    }
    public double gp2GetLeftTrigger() {
        return gamepad2.left_trigger;
    }

    /**
     * The range of the gp1 left trigger cubic press
     * @return
     */
    public boolean gp1GetLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp1LeftTriggerLast && (gp1GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        gp1LeftTriggerLast = (gp1GetLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    public boolean gp1GetLeftTriggerPersistent() {
        boolean isPressedLeftTrigger = false;
        if ((gp1GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        return isPressedLeftTrigger;
    }

    public boolean gp2GetLeftTriggerPersistent() {
        boolean isPressedLeftTrigger = false;
        if ((gp2GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        return isPressedLeftTrigger;
    }

    /**
     * The range of the gp2 left trigger cubic press values
     * @return
     */
    public boolean gp2GetLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp2LeftTriggerLast && (gp2GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        gp2LeftTriggerLast = (gp2GetLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    /**
     * Methods to get the value of gamepad Left Bumper
     *
     * @return gpGamepad1.left_bumper
     * @return gpGamepad2.left_bumper
     */
    public boolean gp1GetLeftBumper() {
        return gamepad1.left_bumper;
    }
    public boolean gp2GetLeftBumper() {
        return gamepad2.left_bumper;
    }

    /**
     * Method to track if Left Bumper was pressed
     * To ensure that the continuous holding of the left bumper does not cause a contiual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing hold or release of button should not trigger action.
     *
     * @return isPressedLeftBumper| = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetLeftBumperPress() {
        boolean isPressedLeftBumper = false;
        if (!gp1LeftBumperLast && gamepad1.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp1LeftBumperLast = gamepad1.left_bumper;
        return isPressedLeftBumper;
    }

    public boolean gp2GetLeftBumperPress() {
        boolean isPressedLeftBumper = false;
        if (!gp2LeftBumperLast && gamepad2.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp2LeftBumperLast = gamepad2.left_bumper;
        return isPressedLeftBumper;
    }

    /**
     * Methods to get the value of gamepad Right Bumper
     *
     * @return gpGamepad1.right_bumper
     * @return gpGamepad2.right_bumper
     */
    public boolean gp1GetRightBumper() {
        return gamepad1.right_bumper;
    }
    public boolean gp2GetRightBumper() {
        return gamepad2.right_bumper;
    }
    /**
     * Method to track if Right Bumper was pressed
     * To ensure that the continuous holding of the right bumper does not cause a continual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedRightBumper = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetRightBumperPress() {
        boolean isPressedRightBumper = false;
        if (!gp1RightBumperLast && gamepad1.right_bumper) {
            isPressedRightBumper = true;
        }
        gp1RightBumperLast = gamepad1.right_bumper;
        return isPressedRightBumper;
    }

    public boolean gp2GetRightBumperPress() {
        boolean isPressedRightBumper = false;
        if (!gp2RightBumperLast && gamepad2.right_bumper) {
            isPressedRightBumper = true;
        }
        gp2RightBumperLast = gamepad2.right_bumper;
        return isPressedRightBumper;
    }

    public boolean gp1GetRightBumperPersistant(){
        return gamepad1.right_bumper;
    }
    public boolean gp2GetRightBumperPersistant(){
        return gamepad2.right_bumper;
    }

    /**
     * Method to track if Button A was pressed
     * To ensure that the continuous holding of Button A does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButton A = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp1ButtonALast && gamepad1.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = gamepad1.a;
        return isPressedButtonA;
    }
    public boolean gp2GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp2ButtonALast && gamepad2.a) {
            isPressedButtonA = true;
        }
        gp2ButtonALast = gamepad2.a;
        return isPressedButtonA;
    }
    public boolean gp1GetA(){
        return gamepad1.a;
    }
    public boolean gp2GetA(){
        return gamepad2.a;
    }

    public GamepadController(Gamepad gamepad1, Gamepad gamepad2, DriveTrain driveTrain) {
    }

    /**
     * Method to track if Button Y was pressed
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonY = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp1ButtonYLast && gamepad1.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = gamepad1.y;
        return isPressedButtonY;
    }
    public boolean gp2GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp2ButtonYLast && gamepad2.y) {
            isPressedButtonY = true;
        }
        gp2ButtonYLast = gamepad2.y;
        return isPressedButtonY;
    }
    public boolean gp1GetY(){
        return gamepad1.y;
    }
    public boolean gp2GetY(){
        return gamepad2.y;
    }

    /**
     * Method to track if Button X was pressed
     * To ensure that the continuous holding of Button X does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonX = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonXPress() {
        boolean isPressedButtonX = false;
        if (!gp1ButtonXLast && gamepad1.x) {
            isPressedButtonX = true;
        }
        gp1ButtonXLast = gamepad1.x;
        return isPressedButtonX;
    }

    public boolean gp2GetButtonXPress() {
        boolean isPressedButtonX = false;
        if (!gp2ButtonXLast && gamepad2.x) {
            isPressedButtonX = true;
        }
        gp2ButtonXLast = gamepad2.x;
        return isPressedButtonX;
    }
    public boolean gp1GetX(){
        return gamepad1.x;
    }
    public boolean gp2GetX(){
        return gamepad2.x;
    }

    /**
     * Method to track if Button B was pressed to move Arm
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonB = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp1ButtonBLast && gamepad1.b) {
            isPressedButtonB = true;
        }
        gp1ButtonBLast = gamepad1.b;
        return isPressedButtonB;
    }
    public boolean gp2GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp2ButtonBLast && gamepad2.b) {
            isPressedButtonB = true;
        }
        gp2ButtonBLast = gamepad2.b;
        return isPressedButtonB;
    }
    public boolean gp1GetB(){
        return gamepad1.b;
    }
    public boolean gp2GetB(){
        return gamepad2.b;
    }

    /**
     * Method to track if Dpad_up was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_up = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp1Dpad_upLast && gamepad1.dpad_up) {
            isPressedDpad_up = true;
        }
        gp1Dpad_upLast = gamepad1.dpad_up;
        return isPressedDpad_up;
    }
    public boolean gp2GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp2Dpad_upLast && gamepad2.dpad_up) {
            isPressedDpad_up = true;
        }
        gp2Dpad_upLast = gamepad2.dpad_up;
        return isPressedDpad_up;
    }

    public boolean gp1GetDpad_up(){
        return gamepad1.dpad_up;
    }
    public boolean gp2GetDpad_up(){
        return gamepad2.dpad_up;
    }

    /**
     * Method to track if Dpad_down was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_down = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp1Dpad_downLast && gamepad1.dpad_down) {
            isPressedDpad_down = true;
        }
        gp1Dpad_downLast = gamepad1.dpad_down;
        return isPressedDpad_down;
    }
    public boolean gp2GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp2Dpad_downLast && gamepad2.dpad_down) {
            isPressedDpad_down = true;
        }
        gp2Dpad_downLast = gamepad2.dpad_down;
        return isPressedDpad_down;
    }

    public boolean gp1GetDpad_down(){
        return gamepad1.dpad_down;
    }
    public boolean gp2GetDpad_down(){
        return gamepad2.dpad_down;
    }

    /**
     * Method to track if Dpad_left was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_left = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_leftPress() {
        boolean isPressedDpad_left;
        isPressedDpad_left = false;
        if (!gp1Dpad_leftLast && gamepad1.dpad_left) {
            isPressedDpad_left = true;
        }
        gp1Dpad_leftLast = gamepad1.dpad_left;
        return isPressedDpad_left;
    }
    public boolean gp2GetDpad_leftPress() {
        boolean isPressedDpad_left;
        isPressedDpad_left = false;
        if (!gp2Dpad_leftLast && gamepad2.dpad_left) {
            isPressedDpad_left = true;
        }
        gp2Dpad_leftLast = gamepad2.dpad_left;
        return isPressedDpad_left;
    }

    public boolean gp1GetDpad_left(){
        return gamepad1.dpad_left;
    }
    public boolean gp2GetDpad_left(){
        return gamepad2.dpad_left;
    }

    /**
     * Method to track if Dpad_right was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_left = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = false;
        if (!gp1Dpad_rightLast && gamepad1.dpad_right) {
            isPressedDpad_right = true;
        }
        gp1Dpad_rightLast = gamepad1.dpad_right;
        return isPressedDpad_right;
    }
    public boolean gp2GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = false;
        if (!gp2Dpad_rightLast && gamepad2.dpad_right) {
            isPressedDpad_right = true;
        }
        gp2Dpad_rightLast = gamepad2.dpad_right;
        return isPressedDpad_right;
    }
    public boolean gp1GetDpad_right(){
        return gamepad1.dpad_right;
    }
    public boolean gp2GetDpad_right(){
        return gamepad2.dpad_right;
    }

    public boolean gp1GetStart(){
        return gamepad1.start;
    }
    public boolean gp2GetStart(){
        return gamepad2.start;
    }

}
