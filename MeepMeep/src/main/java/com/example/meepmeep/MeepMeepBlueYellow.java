package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueYellow {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark());


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( new Pose2d(11.8, 61.7, Math.toRadians(90)))
                                //DRIVE TO BASKET FOR SAMPLE 1
                                .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(225)))
                                //DRIVE TO NEXT SAMPLE FOR SAMPLE 2 INTAKE
                                .lineToLinearHeading(new Pose2d(46.7, 44.5,Math.toRadians(-90)))
                                .waitSeconds(2)
                                //DRIVE TO BASKET FOR SAMPLE 2
                                .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(225)))
                                .waitSeconds(2)
                                //DRIVE TO NEXT SAMPLE FOR SAMPLE 3 INTAKE
                                .lineToLinearHeading(new Pose2d(58.1, 44.5, Math.toRadians(-90)))
                                .waitSeconds(2)
                                //DRIVE TO BASKET FOR SAMPLE 3
                                .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(225)))
                                .waitSeconds(2)
                                //DRIVE TO NEXT SAMPLE FOR SAMPLE 4
                                .lineToLinearHeading(new Pose2d(58.1, 44.5, Math.toRadians(-60)))
                                .waitSeconds(2)
                                //DRIVE TO BASKET FOR SAMPLE 4
                                .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(225)))
                                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}