package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
        import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "strafeAuto")

public class strafeAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder BlueYellow = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-45, -60))
                .strafeTo(new Vector2d(-45, -50))
                .strafeTo(new Vector2d(45, -50));
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