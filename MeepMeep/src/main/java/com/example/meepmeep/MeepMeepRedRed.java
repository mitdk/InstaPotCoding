package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark());


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( new Pose2d(11.8, -60.5, Math.toRadians(270)))
                                //SPECI 1
                                .lineToLinearHeading(new Pose2d(0, -33, Math.toRadians(270)))
                                .waitSeconds(2)
                                //SPECI 2 PICKUP
                                .lineToLinearHeading(new Pose2d(30, -56.8, 0))
                                .waitSeconds(2)
                                //SPECI 2
                                .lineToLinearHeading(new Pose2d(2, -33, Math.toRadians(270)))
                                .waitSeconds(2)
                                //SPECI 3 PICKUP
                                .splineToLinearHeading(new Pose2d(48.6, -45.2, Math.toRadians(90)), Math.toRadians(0))
                                .waitSeconds(2)
                                //SPECI 3 DROPOFF AT HP ZONE
                                .turn(Math.toRadians(180))
                                .waitSeconds(5)
                                //SPECI 3
                                .lineToConstantHeading(new Vector2d(-1, -33))
                                .waitSeconds(2)
                                //PARK
                                .lineToLinearHeading(new Pose2d(56, -60.5, 0))


                                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}