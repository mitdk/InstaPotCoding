package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark());


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(10, -66, 0))

                        .strafeToLinearHeading(new Vector2d(48, -40), Math.toRadians(90))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(58, -40), Math.toRadians(90))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(72, -40), Math.toRadians(90))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(66, -66), Math.toRadians(90));




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}