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


@Autonomous(name = "AutoSample")

public class AutoSample extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-11.8, -61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        LinSlide linslide = new LinSlide(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);


        TrajectoryActionBuilder RedYellow = drive.actionBuilder(initialPose)
                //DRIVE TO BASKET FOR SAMPLE 1


                .afterTime(0, arm.armPerp())
                .afterTime(0,linslide.lsOut())
                .strafeToLinearHeading(new Vector2d(-59.9, -57.3), Math.toRadians(45))
                .waitSeconds(1)
                .afterTime(0,linslide.lsCollect())
                .afterTime(0, arm.armPar())

                //DRIVE TO NEXT SAMPLE FOR SAMPLE 2 INTAKE
                .strafeToLinearHeading(new Vector2d(-48.7, -44.5),Math.toRadians(90))
                .waitSeconds(2)
                //DRIVE TO BASKET FOR SAMPLE 2


                .afterTime(0, arm.armPerp())
                .stopAndAdd(linslide.lsOut())
                .strafeToLinearHeading(new Vector2d(-59, -57.4), Math.toRadians(45))
                .waitSeconds(1)
                .afterTime(0,linslide.lsCollect())
                .afterTime(0, arm.armPar())
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 3 INTAKE
                .strafeToLinearHeading(new Vector2d(-58.2, -44.2), Math.toRadians(90))
                .waitSeconds(2)
                //DRIVE TO BASKET FOR SAMPLE 2
                .afterTime(0, arm.armPerp())
                .stopAndAdd(linslide.lsOut())
                .strafeToLinearHeading(new Vector2d(-57.5, -57.5), Math.toRadians(45))
                .waitSeconds(1)
                .afterTime(0,linslide.lsCollect())
                .afterTime(0, arm.armPar())
                //DRIVE TO NEXT SAMPLE FOR SAMPLE 4
                .strafeToLinearHeading(new Vector2d(-53.4, -44.2), Math.toRadians(120))
                .waitSeconds(2)
                //DRIVE TO BASKET FOR SAMPLE 2
                .afterTime(0, arm.armPerp())
                .stopAndAdd(linslide.lsOut())
                .strafeToLinearHeading(new Vector2d(-55.7, -57.7), Math.toRadians(45))
                .waitSeconds(1)
                .afterTime(0,linslide.lsCollect())

                //DRIVE TO NEXT SAMPLE FOR SAMPLE 4

                //PARK
                .afterTime(0, arm.armAscent())
                .strafeToLinearHeading(new Vector2d(-50,-10), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-23, -10))

                .waitSeconds(20);


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