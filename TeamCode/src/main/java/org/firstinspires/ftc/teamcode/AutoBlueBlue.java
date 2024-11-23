package org.firstinspires.ftc.teamcode;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name = "AutoBlueBlue")


public class AutoBlueBlue extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-11.8, -61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder BlueYellow = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-56, -63, Math.toRadians(51)), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-56.25, -65.15))
                .waitSeconds(1)
                //Brings robot to second sample block, the middle one of the 3 outside the sub
                .strafeToLinearHeading(new Vector2d(-44.7, -56.8), Math.toRadians(77))
              /*  //Intakes 2nd sample block
                .waitSeconds(1)
                //Travel to basket
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                //Deposits 2nd sample block
                .waitSeconds(1)
                //goes to 3rd sample
                .strafeToLinearHeading(new Vector2d(-36.7, -25.8), Math.toRadians(180))
                .lineToX(10)
                //Intakes 3rd sample block
                .waitSeconds(1)
                //Travel to basket
                .lineToX(-10)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                //Deposits 3rd sample block

                //Goes to submersible to pick up 4th sample
                //.lineToLinearHeading(new Pose2d(-56.4,-18.4,Math.toRadians(0)))
                // .splineToLinearHeading(new Pose2d(-25.8, -6.8, 0), Math.toRadians(-90))
           */     //Intakes 4th sample block
                .waitSeconds(26);

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                            BlueYellow.build()










                    )
            );
        }







    }
}