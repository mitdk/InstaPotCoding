package org.firstinspires.ftc.teamcode;

// RR-specific imports

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.LinSlide;
import org.firstinspires.ftc.teamcode.mechanisms.Wrist;


@Autonomous(name = "AutoSpecimen")

public class AutoSpecimenAlt extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, -61.7, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        LinSlide linslide = new LinSlide(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);


        TrajectoryActionBuilder Specimen = drive.actionBuilder(initialPose)
                .afterTime(0,arm.armPerp())
                .afterTime(0,wrist.wristSpecimen())
                .strafeToConstantHeading(new Vector2d(0, -33))
                .afterTime(0,linslide.lsCollect())
                .waitSeconds(2)
                .afterTime(0, claw.clawOpen())
                .afterTime(0, arm.armPar())
                //SPECIS 2 AND 3 PICKUP
                .splineToConstantHeading(new Vector2d(47.4, -8.5),Math.toRadians(0))
                .strafeTo(new Vector2d(47.4, -48.5))
                .splineToConstantHeading(new Vector2d(55, -8.5), Math.toRadians(0))
                .strafeTo(new Vector2d(55, -48.5))
                //SPECI 2
                .strafeToConstantHeading(new Vector2d(47, -38.5))
                .afterTime(0, linslide.lsSpeciCollect())
                .waitSeconds(2)
                .afterTime(0, wrist.wristIntake())
                .afterTime(0, claw.clawClose())
                .afterTime(0, wrist.wristSpecimen())
                .afterTime(0, linslide.lsIn())
                .afterTime(0,arm.armPerp())
                .strafeToLinearHeading(new Vector2d(1, -33), Math.toRadians(270))
                .afterTime(0,linslide.lsCollect())
                .waitSeconds(2)
                .afterTime(0, claw.clawOpen())
                .afterTime(0, arm.armPar())
                //SPECI 3
                .strafeToConstantHeading(new Vector2d(47, -38.5))
                .afterTime(0, linslide.lsSpeciCollect())
                .waitSeconds(2)
                .afterTime(0, wrist.wristIntake())
                .afterTime(0, claw.clawClose())
                .afterTime(0, wrist.wristSpecimen())
                .afterTime(0, linslide.lsIn())
                .afterTime(0,arm.armPerp())
                .strafeToLinearHeading(new Vector2d(1, -33), Math.toRadians(270))
                .afterTime(0,linslide.lsCollect())
                .waitSeconds(2)
                .afterTime(0, claw.clawOpen())
                .afterTime(0, arm.armPar())
                //SPECI 2 PICKUP
                .strafeToLinearHeading(new Vector2d(40, -59.8), Math.toRadians(0))
                .afterTime(0,claw.clawClose())
                .waitSeconds(1)
                .afterTime(0, linslide.lsIn())
                //SPECI 2
                .afterTime(0,arm.armPerp())
                .afterTime(0, wrist.wristSpecimen())
                .strafeToLinearHeading(new Vector2d(1, -33), Math.toRadians(270))
                .afterTime(0,linslide.lsCollect())
                .afterTime(0, claw.clawOpen())
                .waitSeconds(1)

                .afterTime(0, arm.armPar())
                .afterTime(0, linslide.lsSpeciCollect())
                .afterTime(3, wrist.wristIntake())
                //PARK
                .strafeToLinearHeading(new Vector2d(56, -60.5), Math.toRadians(270))
                .waitSeconds(30);
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                            Specimen.build()
                    )
            );
        }
    }
}