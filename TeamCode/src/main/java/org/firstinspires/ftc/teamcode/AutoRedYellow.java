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


@Autonomous(name = "AutoRedYellow")

public class AutoRedYellow extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        LinSlide linslide = new LinSlide(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);


        TrajectoryActionBuilder BlueYellow = drive.actionBuilder(initialPose)
                //DRIVE TO BASKET FOR SAMPLE 1
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2)
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 2 INTAKE
                .strafeToLinearHeading(new Vector2d(-46.7, -44.5),Math.toRadians(90))
                .waitSeconds(2)
                //DRIVE TO BASKET FOR SAMPLE 2
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2)
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 3 INTAKE
                .strafeToLinearHeading(new Vector2d(-58.1, -44.5), Math.toRadians(90))
                .waitSeconds(2)
                //DRIVE TO BASKET FOR SAMPLE 3
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2)
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 4
                .strafeToLinearHeading(new Vector2d(-58.1, -44.5), Math.toRadians(120))
                .waitSeconds(2)
                //DRIVE TO BASKET FOR SAMPLE 4
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45));

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