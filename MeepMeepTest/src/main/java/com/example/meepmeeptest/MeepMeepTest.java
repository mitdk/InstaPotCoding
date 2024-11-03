package com.example.meepmeeptest;

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

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -66, 0))

                        //Moves towards submersible for initial specimen drop
                        .lineToLinearHeading(new Pose2d(3.7, -31.3, Math.toRadians(180)))
                        //Turns wrist servo 45 degrees, between the normal and specimen positions
                        .waitSeconds(0.7)
                        //Mskes specimen touch the bar, brings bot closer
                        .lineToConstantHeading(new Vector2d(3.7, -28))
                        //Turns wrist 45 to complete full 90, specimen should be clipped on
                        //Intake outtakes to let go of specimen
                        .waitSeconds(2)
                        .lineToConstantHeading(new Vector2d(3.7, -33.3))
                        //Brings robot to first sample block, the middle one of the 3 outside the sub
                        .splineToLinearHeading(new Pose2d(36.7, -25.8, Math.toRadians(0)), Math.toRadians(90))
                        //Intakes a sample block
                        .waitSeconds(2)
                        //Travel to basket
                        .lineToConstantHeading(new Vector2d(36.7, -40))
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        //Deposits sample block
                        .waitSeconds(2)
                        //goes to 2nd sample
                        .splineToLinearHeading(new Pose2d(46.7, -25.8, Math.toRadians(0)), Math.toRadians(90))
                        //Intakes 2nd sample block
                        .waitSeconds(2)
                        //Travel to basket
                        .lineToConstantHeading(new Vector2d(36.7, -40))
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        //Deposits 3rd sample block
                        .waitSeconds(2)
                        //goes to 3rd sample
                        .splineToLinearHeading(new Pose2d(56.7, -25.8, Math.toRadians(0)), Math.toRadians(90))
                        //Intakes 3rd sample block
                        .waitSeconds(2)
                        //Travel to basket
                        .lineToConstantHeading(new Vector2d(36.7, -40))
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                                //Deposits 3rd sample block
                        .waitSeconds(2)


                      /*  .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(72, -40, Math.toRadians(90)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(66,-66, Math.toRadians(90)))*/

                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}