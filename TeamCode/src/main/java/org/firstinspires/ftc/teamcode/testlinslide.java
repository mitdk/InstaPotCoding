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
public class testlinslide extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration


        DcMotor armLinSlide = hardwareMap.dcMotor.get("armLinSlide");



        final double LIN_SLIDE_OFF = 0;
        final double LIN_SLIDE_ON = 3900;
        armLinSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        armLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLinSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);









        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            armLinSlide.setTargetPosition(0);
            armLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armLinSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            telemetry.addData("Position:", armLinSlide.getCurrentPosition());
            telemetry.update();
            if (armLinSlide.getCurrentPosition()<=3900){;


                double extend = -gamepad2.right_stick_y;
                armLinSlide.setPower(extend);
            } else if (armLinSlide.getCurrentPosition()>3900){
                armLinSlide.setPower(-0.5);
                }
            }






         //   double extend = gamepad2.right_stick_y;
           // armLinSlide.setPower(extend);



            //OUTTAKE LINEAR SLIDE
            /*if (gamepad2.left_trigger > 0) {
                linSlidePosition = LIN_SLIDE_OFF;

            } else if (gamepad2.right_trigger > 0) {
                linSlidePosition = LIN_SLIDE_ON;
            }
            armLinSlide.setPower(1);
            armLinSlide.setTargetPosition((int) linSlidePosition);
            armLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        }}


