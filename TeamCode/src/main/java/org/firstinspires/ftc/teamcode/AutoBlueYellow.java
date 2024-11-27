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
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        LinSlide linslide = new LinSlide(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);


        TrajectoryActionBuilder samp1 = drive.actionBuilder(initialPose)
                //DRIVE TO BASKET FOR SAMPLE 1
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(225));
                /*.afterTime(0, arm.armPerp())
                .afterTime(0, wrist.wristSpecimen())
                .afterTime(2, linslide.lsOut())
                .stopAndAdd(claw.clawOpen());*/
        TrajectoryActionBuilder samp2 = drive.actionBuilder(new Pose2d(55, 55, Math.toRadians(225)))
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 2 INTAKE
                .strafeToLinearHeading(new Vector2d(46.7, 44.5),Math.toRadians(-90))
                /*.afterTime(3.5, linslide.lsIn())
                .afterTime(3.5, arm.armPar())
                .afterTime(3.5, wrist.wristFlatout())
                .afterTime(5, linslide.lsOut())
                .afterTime(6, wrist.wristIntake())
                .afterTime(6.5, claw.clawClose())*/
                //DRIVE TO BASKET FOR SAMPLE 2
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(225));
                /*.afterTime(7, wrist.wristSpecimen())
                .afterTime(7, linslide.lsIn())
                .afterTime(8, arm.armPerp())
                .afterTime(10, linslide.lsOut())
                .afterTime(11, claw.clawOpen());*/
        TrajectoryActionBuilder samp3 = drive.actionBuilder((new Pose2d(55,55,Math.toRadians(225))))
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 3 INTAKE
                .strafeToLinearHeading(new Vector2d(58.1, 44.5), Math.toRadians(-90))
                /*.afterTime(11.5, linslide.lsIn())
                .afterTime(11.5, wrist.wristFlatout())
                .afterTime(11.5, arm.armPar())
                .afterTime(13, linslide.lsOut())
                .afterTime(14, wrist.wristIntake())
                .afterTime(14.5, claw.clawClose())*/
                //DRIVE TO BASKET FOR SAMPLE 3
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(225));
                /*.afterTime(15, wrist.wristSpecimen())
                .afterTime(15, linslide.lsIn())
                .afterTime(16, arm.armPerp())
                .afterTime(18, linslide.lsOut())
                .afterTime(19, claw.clawOpen());*/
        TrajectoryActionBuilder samp4 = drive.actionBuilder(new Pose2d(55,55,Math.toRadians(225)))
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 4
                .strafeToLinearHeading(new Vector2d(58.1, 44.5), Math.toRadians(-60))
                /*.afterTime(19.5, linslide.lsIn())
                .afterTime(19.5, arm.armPar())
                .afterTime(19.5, wrist.wristFlatout())
                .afterTime(21, linslide.lsOut())
                .afterTime(22, wrist.wristIntake())
                .afterTime(22.5, claw.clawClose())*/
                //DRIVE TO BASKET FOR SAMPLE 4
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(225));
                /*.afterTime(23, wrist.wristSpecimen())
                .afterTime(23, linslide.lsIn())
                .afterTime(24, arm.armPerp())
                .afterTime(26, linslide.lsOut())
                .afterTime(27, claw.clawOpen());*/
        TrajectoryActionBuilder closeOut = samp4.endTrajectory().fresh()
                        .strafeToConstantHeading(new Vector2d(-50.4, 66.6));
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                            samp1.build(),
                            samp2.build(),
                            samp3.build(),
                            samp4.build(),
                            closeOut.build()

                    )
            );
        }
    }
}