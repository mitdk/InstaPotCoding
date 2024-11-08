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
        CRServo intake = hardwareMap.crservo.get("intake");
        Servo outtake = hardwareMap.servo.get("outtake");
        DcMotor armLinSlide = hardwareMap.dcMotor.get("armLinSlide");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        DcMotor linSlideLeft = hardwareMap.dcMotor.get("linSlideLeft");

        final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)
        final double LINEAR_SLIDE_TICKS_PER_DEGREE = 19.8616161616;
        final double INTAKE_COLLECT = -1.0;
        final double INTAKE_OFF = 0.0;
        final double INTAKE_DEPOSIT = 1;
        final double OUTTAKE_COLLECT = 0;
        final double OUTTAKE_DISPOSE = 1;
        /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
        final double ARM_COLLAPSED_INTO_ROBOT = 0;
        final double ARM_COLLECT = -3000;
        final double ARM_DROP = -10;
        final double LIN_SLIDE_OFF = 0;
        final double LIN_SLIDE_ON = 3965;
        double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
        double linSlidePosition = (int) LIN_SLIDE_OFF;
        // Brake code
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
        linSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        //outtake.setDirection(Servo.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        
        linSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideLeft.setTargetPosition(0);
        linSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        intake.setPower(INTAKE_OFF);
        outtake.setPosition(0.5);


        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //DRIVETRAIN
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //ARM LINEAR SLIDE
            double extend = gamepad2.left_stick_y;
            armLinSlide.setPower(extend);

          /*  if (gamepad1.right_trigger > 0) {
                double extend = gamepad2.right_trigger;
                armLinSlide.setPower(extend);
            } else if (gamepad1.left_trigger > 0) {
                double extend = gamepad2.left_trigger;
                armLinSlide.setPower(extend);
            }*/
            //INTAKE
            if (gamepad2.a) {
                intake.setPower(0.5);
            } else if (gamepad2.x) {
                intake.setPower(0);
            } else if (gamepad2.b) {
                intake.setPower(-0.5);

            }
            //OUTTAKE
            if (gamepad2.left_bumper) {
                outtake.setPosition(0.7);


            } else if (gamepad2.right_bumper) {
                outtake.setPosition(0.95);

            }
            //ARM
            if (gamepad2.dpad_up) {
                armPosition = 0;

            } else if (gamepad2.dpad_down) {
                armPosition = 2500;

            } else if (gamepad2.dpad_left) {
                armPosition = 100 ;

            }
            //((DcMotorEx) arm).setVelocity(2100);
            arm.setTargetPosition((int) armPosition);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            //OUTTAKE LINEAR SLIDE
            if (gamepad2.left_trigger > 0) {
                linSlidePosition = LIN_SLIDE_OFF;

            } else if (gamepad2.right_trigger > 0) {
                linSlidePosition = LIN_SLIDE_ON;
            }
            linSlideLeft.setPower(0.95);
            linSlideLeft.setTargetPosition((int) linSlidePosition);
            linSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            /*
            
                */
            //PID STUFF
            telemetry.addData("armTarget: ", arm.getTargetPosition());
            telemetry.addData("arm Encoder: ", arm.getCurrentPosition());
            telemetry.update();
        }

    }
}
