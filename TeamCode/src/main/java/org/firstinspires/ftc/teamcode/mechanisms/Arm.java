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

public class Arm {
    public DcMotorEx linSlide;
    public DcMotorEx Arm_2;


    public Arm(HardwareMap hardwareMap) {
        linSlide = hardwareMap.get(DcMotorEx.class, "arm1");
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlide.setTargetPosition(0);
        linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_2 = hardwareMap.get(DcMotorEx.class, "arm2");
        Arm_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_2.setTargetPosition(0);
        Arm_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public class ArmPerpendicular implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                linSlide.setPower(0.5);
                Arm_2.setPower(0.5);
                initialized = true;
            }

            // checks lift's current position
            double armpos1 = linSlide.getCurrentPosition();
            double armpos2 = Arm_2.getCurrentPosition();
            packet.put("armPos1", armpos1);
            packet.put("armPos2", armpos2);
            if (armpos1 < 450 || armpos2 < 450) {
                return true;
            } else if (armpos2 == 450 || armpos1 == 450) {
                return true;
            } else {
                linSlide.setPower(0);
                Arm_2.setPower(0);
                return false;
            }
        }
    }
    public Action armPerp() {
        return new ArmPerpendicular();
    }
    public class ArmParallel implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                linSlide.setPower(-0.5);
                Arm_2.setPower(-0.5);
                initialized = true;
            }

            // checks lift's current position
            double armpos1 = linSlide.getCurrentPosition();
            double armpos2 = Arm_2.getCurrentPosition();
            packet.put("armPos1", armpos1);
            packet.put("armPos2", armpos2);
            if (armpos1 > 0 || armpos2 > 0) {
                return true;
            } else if (armpos2 == 0 || armpos1 == 0) {
                return true;
            } else {
                linSlide.setPower(0);
                Arm_2.setPower(0);
                return false;
            }
        }
    }
    public Action armPar(){
        return new ArmParallel();
    }
}

