package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.8, -60.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(270))
                .waitSeconds(2)
                //SPECI 2 PICKUP
                .strafeToLinearHeading(new Vector2d(30, -56.8), Math.toRadians(0))
                .waitSeconds(2)
                //SPECI 2
                .strafeToLinearHeading(new Vector2d(4, -33), Math.toRadians(270))
                .waitSeconds(2)
                //SPECI 3 PICKUP
                .splineToLinearHeading(new Pose2d(48.6, -45.2, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(2)
                //SPECI 3 DROPOFF AT HP ZONE
                .turn(Math.toRadians(180))
                .waitSeconds(5)
                //SPECI 3
                .strafeToConstantHeading(new Vector2d(6, -33))
                .waitSeconds(2)
                //PARK
                .strafeToLinearHeading(new Vector2d(56, -60.5), 180)


                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
