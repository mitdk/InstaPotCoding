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
        Servo wrist = hardwareMap.servo.get("wrist");
        CRServo intake = hardwareMap.crservo.get("intake");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        DcMotor linSlideLeft = hardwareMap.dcMotor.get("linSlideLeft");
        DcMotor linSlideRight = hardwareMap.dcMotor.get("linSlideRight");

        final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)
        final double LINEAR_SLIDE_TICKS_PER_DEGREE = 19.8616161616;
        final double INTAKE_COLLECT = -1.0;
        final double INTAKE_OFF = 0.0;
        final double INTAKE_DEPOSIT = 1;
        /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
        final double WRIST_FOLDED_IN = 0.945;
        final double WRIST_FOLDED_OUT = 0.6;
        final double ARM_COLLAPSED_INTO_ROBOT = 0;
        final double ARM_COLLECT = -3000;
        final double ARM_DROP = -10;
        double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
        // Brake code
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) arm).setCurrentAlert(0.1, CurrentUnit.AMPS);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);


        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

            double extend = -gamepad2.right_stick_y;
            linSlideLeft.setPower(extend);
            linSlideRight.setPower(extend);

            if (gamepad2.a) {
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad2.x) {
                intake.setPower(0);
            } else if (gamepad2.b) {
                intake.setPower(-1);
            }

            if (gamepad2.left_bumper) {
                wrist.setPosition(WRIST_FOLDED_OUT);
            } else if (gamepad2.right_bumper) {
                wrist.setPosition(WRIST_FOLDED_IN);
            }

/*
            if (gamepad2.dpad_up){

                ((DcMotorEx) arm).setVelocity(1);


            } else if (gamepad2.dpad_down){
                armPosition = ARM_DROP;
            } else if (gamepad2.dpad_left){
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
            }
            arm.setTargetPosition((int) armPosition);
            ((DcMotorEx) arm).setVelocity(200);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (((DcMotorEx) arm).isOverCurrent()){
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }

*/
            if (gamepad2.dpad_up) {
                armPosition = 0;


                 // Move up by a set increment
            } else if (gamepad2.dpad_down) {
                armPosition = -3400;

            } else if (gamepad2.dpad_left) {
                armPosition = 100 ;

            }
            ((DcMotorEx) arm).setVelocity(2100);
            arm.setTargetPosition((int) armPosition);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            /*if (arm.isBusy()) {

            } else {
                arm.setPower(0);
            }
*/

            telemetry.addData("armTarget: ", arm.getTargetPosition());
            telemetry.addData("arm Encoder: ", arm.getCurrentPosition());
            telemetry.update();
        }

    }
}