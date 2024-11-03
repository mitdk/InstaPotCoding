package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
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
                        drive.trajectorySequenceBuilder(new Pose2d(-10, -60, Math.toRadians(90)))
                                //RED SIDE RED SAMPLE
                                //Brings the robot to the basket for first sample
                                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                                .waitSeconds(1)
                                //Brings robot to second sample block, the middle one of the 3 outside the sub

                                .splineToLinearHeading(new Pose2d(36.7,-25.8, Math.toRadians(180)), Math.toRadians(90))

                                //Intakes 2nd sample block
                                .waitSeconds(1)
                                //Travel to basket
                                .lineToConstantHeading(new Vector2d(36.7, -40.8))
                                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                                //Deposits 2nd sample block
                                .waitSeconds(1)
                                //goes to 3rd sample
                                .splineToLinearHeading(new Pose2d(48,-25.8, Math.toRadians(180)), Math.toRadians(90))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(48, -40.8))
                                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(58, -61, Math.toRadians(180)))

                                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
