package org.firstinspires.ftc.teamcode.OpModes.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.SampleColorLight;

public class ColorLightTest extends LinearOpMode {
    private SampleColorLight colorLights;

    @Override
    public void runOpMode() throws InterruptedException {
        colorLights = new SampleColorLight(this);

        waitForStart();
        while(!isStopRequested()) {
            telemetry.update();
            while(opModeIsActive()) {
                colorLights.runSampleColorDetection();
            }
        }
    }
}



