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

        final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)
        final double LINEAR_SLIDE_TICKS_PER_DEGREE = 19.8616161616;
        final double INTAKE_COLLECT = 0.1;
        final double INTAKE_DEPOSIT = 0.3;
        final double WRIST_PICKUP = -1.0;
        double armPosition = 0;
        // Brake code
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
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
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower*0.9);
            backLeftMotor.setPower(backLeftPower*0.9);
            frontRightMotor.setPower(frontRightPower*0.9);
            backRightMotor.setPower(backRightPower*0.9);

            armLinSlide.setTargetPosition(0);
            armLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLinSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            telemetry.addData("Position:", armLinSlide.getCurrentPosition());
            telemetry.update();
            if (armLinSlide.getCurrentPosition() <= 3120){
                double extend = -gamepad2.right_stick_y;
                armLinSlide.setPower(extend);
            } else if (armLinSlide.getCurrentPosition() > 3120){
                armLinSlide.setPower(-0.5);
            }
            //SLOW MOVING DRIVETRAIN
            if (gamepad1.dpad_up){
                frontLeftMotor.setPower(0.4);
                backLeftMotor.setPower(0.4);
                frontRightMotor.setPower(0.4);
                backRightMotor.setPower(0.4);
            }
            if (gamepad1.dpad_down){
                frontLeftMotor.setPower(-0.4);
                backLeftMotor.setPower(-0.4);
                frontRightMotor.setPower(-0.4);
                backRightMotor.setPower(-0.4);
            }
            //CLAW
            if (gamepad2.right_bumper) {
                claw.setPosition(INTAKE_COLLECT);
            } else if (gamepad2.left_bumper) {
                claw.setPosition(INTAKE_DEPOSIT);
            }

            if (gamepad2.a) {
                wrist.setPosition(0);
            } else if (gamepad2.b) {
                wrist.setPosition(0.5);
            } else if  (gamepad2.x) {
                wrist.setPosition(0.9);
            }

            //ARM
            if (gamepad2.dpad_up) {
                //RESET AND INTAKE
                armPosition = 0;

            } else if (gamepad2.dpad_left) {
                //OUTTAKE PERPENDICULAR
                armPosition = 900 ;

            } else if (gamepad2.dpad_right){
                //OUTTAKE SPECIMEN
                armPosition = 450;
            }
            ((DcMotorEx) arm1).setVelocity(2100);
            ((DcMotorEx) arm2).setVelocity(2100);
            arm1.setTargetPosition((int) armPosition);
            arm2.setTargetPosition((int) armPosition);
            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /*
            
                */
            //PID STUFF
            //telemetry.addData("armTarget: ", arm.getTargetPosition());
            //telemetry.addData("arm Encoder: ", arm.getCurrentPosition());
            //telemetry.update();
        }

    }
}
