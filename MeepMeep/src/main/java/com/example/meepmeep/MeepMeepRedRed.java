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

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder( new Pose2d(11.8, -61.7, Math.toRadians(90)))
                        .lineToConstantHeading(new Vector2d(0, -33))
                        .lineToLinearHeading(new Pose2d(40, -59.8, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(1, -33, Math.toRadians(270)))
                        .splineToLinearHeading(new Pose2d(36.6, -25.1, Math.toRadians(270)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(47.4, -8.5),Math.toRadians(0))
                        .forward(40)
                        .splineToConstantHeading(new Vector2d(55, -8.5), Math.toRadians(0))
                        .forward(40)
                        .lineToConstantHeading(new Vector2d(47, -38.5))
                        .lineToLinearHeading(new Pose2d(3, -33,Math.toRadians(-90)))
                        .lineToConstantHeading(new Vector2d(47, -38.5))
                        .lineToConstantHeading(new Vector2d(0, -33))
                        .lineToLinearHeading(new Pose2d(56, -60.5, Math.toRadians(270)))
                        


                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}