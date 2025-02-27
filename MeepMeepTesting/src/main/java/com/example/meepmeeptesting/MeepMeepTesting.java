package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(300);

        final Pose2d startPose = new Pose2d(new Vector2d(-2, 60), 90);
         final Vector2d dropPose = new Vector2d(-55, -50); //132
        //  private final Vector2d dropPoseAdjust = new Vector2d(10, 33.5); //132
        // private final Vector2d dropPoseAdjust2 = new Vector2d(17, 30);
       //  final Vector2d midPoseCycle = new Vector2d(-46, -50); //90
         final Vector2d cycle1 = new Vector2d(-41, -39); //50
         final Vector2d cycle2 = new Vector2d(-53, -39); //40
         final Vector2d midPoseSub = new Vector2d(-52, -45); //90
         final Vector2d parkPose = new Vector2d(-27, -13); //90

        final Vector2d hangPose = new Vector2d(0,-33);
        final Vector2d midPoseSpike = new Vector2d(38,-33);
        final Vector2d midPoseSpike2 = new Vector2d(45,-12);
        final Vector2d spikeCycle1 = new Vector2d(45,-68);
        final Vector2d spike2 = new Vector2d(58,-12);
        final Vector2d spikeCycle2 = new Vector2d(58,-68);
        final Vector2d midPoseObserv = new Vector2d(58,-40);





        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -72, 90))
                .strafeTo(dropPose)
                .turn(Math.toRadians(135))
          //      .strafeTo(midPoseCycle)
                .turn(Math.toRadians(-135))
                .strafeTo(cycle1)
                .turn(Math.toRadians(40))
                .strafeTo(dropPose)
                .turn(Math.toRadians(95))
           //     .strafeTo(midPoseCycle)
                .strafeTo(cycle2)
                .turn(Math.toRadians(-135))
                .turn(Math.toRadians(40))
                .strafeTo(dropPose)
                .turn(Math.toRadians(95))
                .turn(Math.toRadians(-55))
                .strafeTo(midPoseSub)
                .strafeTo(parkPose)
                      // .forward(30)
                        //.turn(Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        //basket
                      //  .turn(Math.toRadians(180))
                        /*.forward(48)
                        .strafeTo(new Vector2d(0.0, -37.0))
                        .turn(Math.toRadians(-90))
                        .strafeTo(new Vector2d(-48.0, -58.0))
                        .turn(Math.toRadians(90)) */
                        .build());

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark() )
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -72, 0))
                        .turn(Math.toRadians(90)) // adding to meepmeep so virtual robot is forward
                        .strafeTo(hangPose)
                        .strafeTo(midPoseSpike)
                        .strafeTo(midPoseSpike2)
                        .strafeTo(spikeCycle1)
                        .strafeTo(midPoseSpike2)
                        .strafeTo(spike2)
                        .strafeTo(spikeCycle2)
                        .strafeTo(midPoseObserv)
                        .turn(Math.toRadians(180))
                        .strafeTo(spikeCycle2)
                        .strafeTo(hangPose)
                        .turn(Math.toRadians(180))
                        .strafeTo(midPoseObserv)
                        .turn(Math.toRadians(180))
                        .strafeTo(spikeCycle2)
                        .strafeTo(hangPose)
                        .turn(Math.toRadians(180))
                        .strafeTo(spikeCycle2)

                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();


    }
}