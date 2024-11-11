package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.jetbrains.annotations.NotNull;


@Autonomous(name = "EmptyAutoMode")

public class emptyAuto extends LinearOpMode {
    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftfront");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("rightfront");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("leftback");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("rightback");
    CRServo intake = hardwareMap.crservo.get("intake");
    Servo outtake = hardwareMap.servo.get("outtake");
    DcMotor armLinSlide = hardwareMap.dcMotor.get("armLinSlide");
    DcMotor arm = hardwareMap.dcMotor.get("arm");
    DcMotor linSlideLeft = hardwareMap.dcMotor.get("linSlideLeft");

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) arm).setCurrentAlert(0.1, CurrentUnit.AMPS);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linSlideLeft.setTargetPosition(0);
        linSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setPower(0);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            outtake.setPosition(0.9);
            sleep(30000);
            telemetry.addData("running", 0);
            telemetry.update();


        }

    }
}