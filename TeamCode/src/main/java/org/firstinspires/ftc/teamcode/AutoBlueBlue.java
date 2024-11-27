package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.LinSlide;
import org.firstinspires.ftc.teamcode.mechanisms.Wrist;


@Autonomous(name = "AutoBlueBlue")

public class AutoBlueBlue extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, -61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        LinSlide linslide = new LinSlide(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);


        TrajectoryActionBuilder RedYellow = drive.actionBuilder(initialPose)
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
                .strafeToLinearHeading(new Vector2d(56, -60.5), Math.toRadians(180));
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                            RedYellow.build()
                    )
            );
        }
    }
}