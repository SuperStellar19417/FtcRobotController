package org.firstinspires.ftc.teamcode.OpModes.practiceOpmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

@Disabled
public class PracticeOpMode extends LinearOpMode {
    private DcMotorEx testMotor;
    private Servo testServo;
    private GamepadController gamepad;





    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
        testServo = hardwareMap.get(Servo.class, "testServo");
        waitForStart();

        if (gamepad.gp1GetRightBumperPress()) {
          runToIn();
          telemetry.addData("Motor is in", testMotor);

        }
        if (gamepad.gp1GetLeftBumperPress() ) {

            runToOut();
            telemetry.addData("Motor is out", testMotor);

        }
        if(gamepad.gp1GetButtonBPress()) {


        }

        if (isStopRequested()){

            return;
        }


        while(!isStopRequested()){

            runToIn();
            runToOut();

        }

    }

    public void runToIn(){
        testMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        testMotor.setTargetPosition(1000);
        testMotor.setPower(10);



    }
    public void runToOut(){
        testMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        testMotor.setTargetPosition(2000);
        testMotor.setPower(10);


    }
    public void runUp() {
        testServo.setDirection(Servo.Direction.FORWARD);
        testServo.scaleRange(0.0,0.5);

    }
    public void runDown(){
        testServo.setDirection(Servo.Direction.REVERSE);
        testServo.scaleRange(0.5,1.0);

    }

}
