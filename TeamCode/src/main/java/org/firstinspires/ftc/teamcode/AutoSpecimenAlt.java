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
import org.firstinspires.ftc.teamcode.mechanisms.ArmPIDF;
import org.opencv.core.Mat;


@Autonomous(name = "AutoSpecimenAlt")

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
                .afterTime(0,claw.clawClose())
                .afterTime(0, wrist.wristSpecimen())
                .afterTime(0,arm.armPerp())
                .afterTime(0.1, linslide.lsSpeciCollect())
                .strafeToConstantHeading(new Vector2d(11, -38.7))
                .afterTime(0,linslide.lsOut())
                .waitSeconds(0.5)

                .afterTime(0, claw.clawOpen())




               //SPECIS 2 AND 3 PICKUP

                .strafeToLinearHeading(new Vector2d(41.4,-40), Math.toRadians(270))
                .afterTime(0, linslide.lsIn())

                .afterTime(0, arm.armPar())
                .afterTime(0, wrist.wristIntake())
                .afterTime(0, claw.clawOpen())

                .strafeTo(new Vector2d(37, -10))
                .strafeTo(new Vector2d(50, -10))
                .strafeTo(new Vector2d(50, -53.425))
                .strafeTo(new Vector2d(50, -10))
                .strafeTo(new Vector2d(60, -10))

                .strafeTo(new Vector2d(60, -56.5))
                .waitSeconds(0.2)
                .afterTime(0, claw.clawClose())
                .waitSeconds(0.2)

                .afterTime(0, wrist.wristSpecimen())

                .afterTime(0, linslide.lsSpeciCollect())
                .afterTime(0,arm.armPerp())
                .strafeToConstantHeading(new Vector2d(2, -38.5))
                .afterTime(0,linslide.lsOut())
                .waitSeconds(0.35)


                .afterTime(0, claw.clawOpen())
                .afterTime(0, linslide.lsIn())
                .afterTime(0, arm.armPar())
                .afterTime(0, wrist.wristIntake())
                .afterTime(0, claw.clawOpen())
                .strafeTo(new Vector2d(57, -57.15))
                .waitSeconds(0.2)
                .afterTime(0, claw.clawClose())
                .waitSeconds(0.3)

                .afterTime(0, wrist.wristSpecimen())

                .afterTime(0, linslide.lsSpeciCollect())
                .afterTime(0,arm.armPerp())
                .strafeToConstantHeading(new Vector2d(4, -39))
                .afterTime(0,linslide.lsOut())
                .waitSeconds(0.35)


                .afterTime(0, claw.clawOpen())
                .afterTime(0, linslide.lsIn())
                .afterTime(0, wrist.wristIntake())
                .afterTime(0, arm.armPar())
                .afterTime(0, claw.clawOpen())
                .strafeTo(new Vector2d(57, -57.45))


                .waitSeconds(0.2)
                .afterTime(0, claw.clawClose())
                .waitSeconds(0.2)

                .afterTime(0, wrist.wristSpecimen())

                .afterTime(0, linslide.lsSpeciCollect())
                .afterTime(0,arm.armPerp())
                .strafeToConstantHeading(new Vector2d(6, -40.5))
                .afterTime(0,linslide.lsOut())
                .waitSeconds(0.35)


                .afterTime(0, claw.clawOpen())
                .afterTime(0, linslide.lsIn())
                .afterTime(0, arm.armPar())



             /*   .afterTime(0,arm.armAscent())
                .afterTime(0, linslide.lsSpeciCollect())

                .strafeToLinearHeading(new Vector2d(5,-28), Math.toRadians(90))
                .afterTime(0, wrist.wristFlatout())
                .afterTime(0, arm.armSpeciRelease())
                .waitSeconds(0.5)
                .afterTime(0,claw.clawOpen())
                .strafeToLinearHeading(new Vector2d(61, -47), Math.toRadians(270))

                .afterTime(0, wrist.wristIntake())
                .afterTime(0, claw.clawOpen())
                .waitSeconds(1)
                .afterTime(0, claw.clawClose())
                .waitSeconds(1)

                .afterTime(0,arm.armAscent())
                .afterTime(0, linslide.lsSpeciCollect())

                .strafeToLinearHeading(new Vector2d(5,-28), Math.toRadians(90))
                .afterTime(0, wrist.wristFlatout())
                .afterTime(0, arm.armSpeciRelease())
                //.waitSeconds(0.2)
            */    //.afterTime(0, claw.clawOpen())




                //SPECI 2

            /*    .afterTime(0, linslide.lsSpeciCollect())
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
                //PARK*/
                .strafeToLinearHeading(new Vector2d(56, -59), Math.toRadians(270))
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