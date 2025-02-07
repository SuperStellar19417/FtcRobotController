package org.firstinspires.ftc.teamcode.OpModes.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.ColorLights;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

public class ColorLightTest extends LinearOpMode {
    private ColorLights colorLights;

    @Override
    public void runOpMode() throws InterruptedException {
        colorLights = new ColorLights(this);

        waitForStart();
        while(!isStopRequested()) {
            telemetry.update();
            while(opModeIsActive()) {
                colorLights.setHlTopColor();
            }
        }
    }
}



