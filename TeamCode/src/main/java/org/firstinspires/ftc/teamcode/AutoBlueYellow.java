package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.ParallelAction;
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


@Autonomous(name = "AutoBlueYellow")

public class AutoBlueYellow extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-11.8, -61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        LinSlide linslide = new LinSlide(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);


        TrajectoryActionBuilder samp1 = drive.actionBuilder(initialPose)
                //DRIVE TO BASKET FOR SAMPLE 1
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(225))
                .afterTime(0, arm.armPerp())
                .afterTime(0, wrist.wristSpecimen())
                .afterTime(1, linslide.lsOut())
                .stopAndAdd(claw.clawOpen());
        TrajectoryActionBuilder samp2 = drive.actionBuilder(new Pose2d(55, 55, 225))
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 2 INTAKE
                .strafeToLinearHeading(new Vector2d(46.7, 44.5),Math.toRadians(-90))
                .waitSeconds(2)
                //DRIVE TO BASKET FOR SAMPLE 2
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(225))
                .waitSeconds(2);
        TrajectoryActionBuilder samp3 = drive.actionBuilder((new Pose2d(55,55,225)))
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 3 INTAKE
                .strafeToLinearHeading(new Vector2d(58.1, 44.5), Math.toRadians(-90))
                .waitSeconds(2)
                //DRIVE TO BASKET FOR SAMPLE 3
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(225))
                .waitSeconds(2);
        TrajectoryActionBuilder samp4 = drive.actionBuilder((new Pose2d(55,55,225)))
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 4
                .strafeToLinearHeading(new Vector2d(58.1, 44.5), Math.toRadians(-60))
                .waitSeconds(2)
                //DRIVE TO BASKET FOR SAMPLE 4
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(225));
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(

                    )
            );
        }
    }
}