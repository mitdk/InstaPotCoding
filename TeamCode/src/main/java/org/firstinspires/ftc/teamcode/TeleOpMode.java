package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.LinSlide;

@TeleOp
public class TeleOpMode extends LinearOpMode {
    //PID CONTROLLER
    private PIDFController controller;

    public static double p = 0.003, i = 0.001, d = 0.0001;

    public static double f = 0.0001;


    public static int target = 0;

    private final double TICKS_PER_DEGREE = 5281.1 / 360;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftfront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("rightfront");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("leftback");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightback");
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");
        DcMotor linSlide = hardwareMap.dcMotor.get("armLinSlide");
        DcMotor arm1 = hardwareMap.dcMotor.get("arm1");
        DcMotor arm2 = hardwareMap.dcMotor.get("arm2");
        double armPosition = 0;
        final double INTAKE_DEPOSIT = 0.23;
        final double INTAKE_COLLECT = 0.05;
        final double WRIST_PICKUP = 0.03;
        final double WRIST_SPECIMEN = 0.26;
        final double WRIST_SPECICOLLECT = 0.2;
        final double WRIST_FLATOUT = 0.05;
        final double WRIST_SPECIMEN_RAM = (((90/1320)*armPosition) * (0.26/180) + 0.015);
        //
        // Brake code
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        linSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlide.setTargetPosition(0);
        linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setTargetPosition(0);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setTargetPosition(0);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);





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

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //LINEAR SLIDE
            if (gamepad2.right_stick_y !=0 ) {
                linSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (linSlide.getCurrentPosition() <= 2000 ) {
                    double extend = -gamepad2.right_stick_y;
                    linSlide.setPower(extend);
                } else if (linSlide.getCurrentPosition() > 2000 && arm1.getCurrentPosition() < 500)  {
                    linSlide.setPower(-0.5);
                }
            }

//************************************************************************************************************************************************************

            //CLAW
            if (gamepad2.left_bumper) {

                claw.setPosition(INTAKE_DEPOSIT);
            } else if (gamepad2.right_bumper) {
                claw.setPosition(INTAKE_COLLECT);
            }
            //specideposit
            if (gamepad1.dpad_up) {
                linSlide.setPower(1);
                linSlide.setTargetPosition(900);
                linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                claw.setPosition(INTAKE_COLLECT);
                wrist.setPosition(0.015);
                linSlide.setPower(1);
                linSlide.setTargetPosition(0);
                linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPosition = 0;
                sleep(250);
                ((DcMotorEx) arm1).setVelocity(1450);
                ((DcMotorEx) arm2).setVelocity(1450);
                arm1.setTargetPosition((int) armPosition);
                arm2.setTargetPosition((int) armPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(WRIST_PICKUP);

            }
            //WRIST
            if (gamepad2.a) {
                wrist.setPosition(WRIST_SPECIMEN);
            } else if (gamepad2.y) {
                wrist.setPosition(WRIST_PICKUP);
            } else if (gamepad2.b) {
                wrist.setPosition(WRIST_FLATOUT);
            } else if (gamepad2.x) {
                wrist.setPosition(0.11);
            }
            telemetry.addData("ArmPos1", arm1.getCurrentPosition());
            telemetry.addData("ArmPos2", arm2.getCurrentPosition());
            telemetry.update();
            //ARM

            if (gamepad2.dpad_up) {
                //INTAKE PARALLEL
                wrist.setPosition(WRIST_PICKUP);
                armPosition = 1320;
                ((DcMotorEx) arm1).setVelocity(1500);
                ((DcMotorEx) arm2).setVelocity(1500);
                arm1.setTargetPosition((int) armPosition);
                arm2.setTargetPosition((int) armPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.dpad_down) {
                //OUTTAKE PERPENDICULAR
                claw.setPosition(INTAKE_COLLECT);
                wrist.setPosition(0.09);
                armPosition = 0;
                ((DcMotorEx) arm1).setVelocity(1500);
                ((DcMotorEx) arm2).setVelocity(1500);
                arm1.setTargetPosition((int) armPosition);
                arm2.setTargetPosition((int) armPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                linSlide.setPower(1);
                linSlide.setTargetPosition(590);
                linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                claw.setPosition(INTAKE_DEPOSIT);
                sleep(250);
                armPosition = 1320;
                ((DcMotorEx) arm1).setVelocity(1500);
                ((DcMotorEx) arm2).setVelocity(1500);
                arm1.setTargetPosition((int) armPosition);
                arm2.setTargetPosition((int) armPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linSlide.setPower(1);
                linSlide.setTargetPosition(770);
                linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(WRIST_PICKUP);


            } else if (gamepad2.dpad_right) {
                //SPECIMEN OUTTAKE
                armPosition = 610;
                ((DcMotorEx) arm1).setVelocity(1500);
                ((DcMotorEx) arm2).setVelocity(1500);
                arm1.setTargetPosition((int) armPosition);
                arm2.setTargetPosition((int) armPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(WRIST_FLATOUT);
            /*    linSlide.setPower(1);
                linSlide.setTargetPosition(1350);
                linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

            } else if (gamepad2.dpad_left) {
                //RESET AND INTAKE
                claw.setPosition(0.2);
                wrist.setPosition(0.13);
                linSlide.setPower(1);
                linSlide.setTargetPosition(0);
                linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPosition = 0;
                sleep(750);
                ((DcMotorEx) arm1).setVelocity(1450);
                ((DcMotorEx) arm2).setVelocity(1450);
                arm1.setTargetPosition((int) armPosition);
                arm2.setTargetPosition((int) armPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(WRIST_PICKUP);
            }
        }
    }
}