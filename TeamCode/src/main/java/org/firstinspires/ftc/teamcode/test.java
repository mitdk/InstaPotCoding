package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "test")
public class test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Set initial position
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));

        // Initialize MecanumDrive with the hardware map and initial pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Build trajectory
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(3.7, -31.2), Math.toRadians(180))
                .waitSeconds(0.7)
                .strafeToConstantHeading(new Vector2d(3.7, -28))
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(3.7, -33.3))
                .splineToLinearHeading(new Pose2d(36.7, -25.8, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(36.7, -40))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180));

        telemetry.addLine("Robot Initialized. Ready to start.");
        telemetry.update();

        // Wait for the start command
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Execute the action sequence
            Actions.runBlocking(
                    new SequentialAction(
                            tab1.build()
                    )
            );

            telemetry.addLine("Trajectory Complete");
            telemetry.update();

            // Exit the loop after the action sequence to avoid redundant execution
            break;
        }
    }
}