package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(20, 20, Math.toRadians(141.1), Math.toRadians(30), 13.21)
                .setDimensions(14.25, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(270) ) )
                                .forward(32)
                                .back(2)
                                .strafeRight(18)
                                .forward(21)
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(48, 12))
                                .strafeLeft(24)
                                .lineTo(new Vector2d( 57, 38))
                                .addDisplacementMarker( () -> {
                                    //lever.setPosition(0.68);
                                    /// sleep(1000);
                                })
                                .back(4)
                                .addDisplacementMarker( () -> {
                                    //lever.setPosition(0.6);
                                    //sleep(1000);
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}