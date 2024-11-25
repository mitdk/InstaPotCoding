package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PController;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class TeleOpMode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration


        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftfront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("rightfront");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("leftback");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightback");
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");
        DcMotor armLinSlide = hardwareMap.dcMotor.get("armLinSlide");
        DcMotor arm1 = hardwareMap.dcMotor.get("arm1");
        DcMotor arm2 = hardwareMap.dcMotor.get("arm2");

        final double INTAKE_COLLECT = 0.0;
        final double INTAKE_DEPOSIT = 0.2;
        final double WRIST_PICKUP = 0.65;
        final double WRIST_SPECIMEN = 0;
        final double WRIST_FLATOUT = 0.35;
        double armPosition = 0;
        // Brake code
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        armLinSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setTargetPosition(0);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setTargetPosition(0);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw.setPosition(-0.1);
        wrist.setPosition(0);


        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //DRIVETRAIN
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.right_stick_x;
            double rx = gamepad1.left_stick_x * 1.1;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x - rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x + rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower * 0.9);
            backLeftMotor.setPower(backLeftPower * 0.9);
            frontRightMotor.setPower(frontRightPower * 0.9);
            backRightMotor.setPower(backRightPower * 0.9);


            armLinSlide.setTargetPosition(0);
            armLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLinSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            telemetry.addData("Position:", armLinSlide.getCurrentPosition());
            telemetry.update();
            if (armLinSlide.getCurrentPosition() <= 3120) {
                double extend = -gamepad2.right_stick_y;
                armLinSlide.setPower(extend);
            } else if (armLinSlide.getCurrentPosition() > 3120) {
                armLinSlide.setPower(-0.5);
            }
            //SLOW MOVING DRIVETRAIN
            if (gamepad1.dpad_up) {
                frontLeftMotor.setPower(0.4);
                backLeftMotor.setPower(0.4);
                frontRightMotor.setPower(0.4);
                backRightMotor.setPower(0.4);
            }
            if (gamepad1.dpad_down) {
                frontLeftMotor.setPower(-0.4);
                backLeftMotor.setPower(-0.4);
                frontRightMotor.setPower(-0.4);
                backRightMotor.setPower(-0.4);
            }
            //CLAW
            if (gamepad2.left_bumper) {
                claw.setPosition(INTAKE_COLLECT);
            } else if (gamepad2.right_bumper) {
                claw.setPosition(INTAKE_DEPOSIT);
            }

            if (gamepad2.a) {
                wrist.setPosition(WRIST_SPECIMEN);
            } else if (gamepad2.b) {
                wrist.setPosition(WRIST_PICKUP);
            } else if (gamepad2.y) {
                wrist.setPosition(WRIST_FLATOUT);
            }

            //ARM
            if (gamepad2.dpad_down) {
                //RESET AND INTAKE
                armPosition = 0;
                ((DcMotorEx) arm1).setVelocity(250);
                ((DcMotorEx) arm2).setVelocity(250);
                arm1.setTargetPosition((int) armPosition);
                arm2.setTargetPosition((int) armPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.dpad_up) {
                //OUTTAKE PERPENDICULAR
                armPosition = 450;
                ((DcMotorEx) arm1).setVelocity(500);
                ((DcMotorEx) arm2).setVelocity(500);
                arm1.setTargetPosition((int) armPosition);
                arm2.setTargetPosition((int) armPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.dpad_right) {
                armPosition = 200;
                ((DcMotorEx) arm1).setVelocity(500);
                ((DcMotorEx) arm2).setVelocity(500);
                arm1.setTargetPosition((int) armPosition);
                arm2.setTargetPosition((int) armPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            /*

             */
            //PID STUFF
            //telemetry.addData("armTarget: ", arm.getTargetPosition());
            //telemetry.addData("arm Encoder: ", arm.getCurrentPosition());
            //telemetry.update();
        }

    }}
