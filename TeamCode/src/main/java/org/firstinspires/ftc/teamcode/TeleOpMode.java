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
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double tick_in_degree = 700/180.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        controller = new PIDController (p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



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
        final double INTAKE_COLLECT    = -1.0;
        final double INTAKE_OFF        =  0.0;
        final double INTAKE_DEPOSIT    =  0.5;
        /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
        final double WRIST_FOLDED_IN   = 0.945;
        final double WRIST_FOLDED_OUT  = 0.65;
        final double ARM_COLLAPSED_INTO_ROBOT  = .110 * ARM_TICKS_PER_DEGREE;
        final double ARM_COLLECT = -.40 * ARM_TICKS_PER_DEGREE;
        final double ARM_DROP = -.10 * ARM_TICKS_PER_DEGREE;
        double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
        // Brake code
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        ((DcMotorEx) arm).setCurrentAlert(0.1,CurrentUnit.AMPS);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);
        controller.setPID(p, i, d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos (Math.toRadians(target/tick_in_degree)) * f;
        double power = pid + ff;
        arm.setPower(power);
        telemetry.addData ("pos", armPos);
        telemetry.addData ("target", target);
        telemetry.update();

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = - (y - x + rx) / denominator;
            double backLeftPower = - (y + x + rx) / denominator;
            double frontRightPower = - (y - x - rx) / denominator;
            double backRightPower = - (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            double extend = -gamepad2.right_stick_y;
            linSlideLeft.setPower(extend);
            linSlideRight.setPower(extend);

            if (gamepad2.a) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad2.x) {
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad2.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }

            if (gamepad2.left_bumper) {
                wrist.setPosition(WRIST_FOLDED_OUT);
            }
            else if (gamepad2.right_bumper) {
                wrist.setPosition(WRIST_FOLDED_IN);
            }


            if (gamepad2.dpad_up){
                armPosition = ARM_COLLECT;

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


        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("armTarget: ", arm.getTargetPosition());
        telemetry.addData("arm Encoder: ", arm.getCurrentPosition());
        telemetry.update();

    }
}