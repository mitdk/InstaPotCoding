package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.mechanisms.Wrist;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinSlide {
    public DcMotorEx linSlide;


    public LinSlide(HardwareMap hardwareMap) {
        linSlide = hardwareMap.get(DcMotorEx.class, "arm1");
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlide.setTargetPosition(0);
        linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public class LinSlideOut implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                linSlide.setPower(0.5);
                initialized = true;
            }

            // checks lift's current position
            double linSlidePos = linSlide.getCurrentPosition();
            packet.put("linSlidePos", linSlidePos);
            if (linSlidePos < 3965) {
                return true;
            } else {
                linSlide.setPower(0);
                return false;
            }
        }
    }
    public Action lsOut() {
        return new LinSlideOut();
    }
    public class LinSlideIn implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                linSlide.setPower(-0.5);
                initialized = true;
            }

            // checks lift's current position
            double linSlidePos = linSlide.getCurrentPosition();
            packet.put("linSlidePos", linSlidePos);
            if (linSlidePos > 0) {
                return true;
            } else {
                linSlide.setPower(0);
                return false;
            }
        }
    }
    public Action lsIn(){
        return new LinSlideIn();
    }
}

