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

import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.LinSlide;
import org.firstinspires.ftc.teamcode.mechanisms.Wrist;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    public DcMotorEx Arm_1;
    public DcMotorEx Arm_2;


    public Arm(HardwareMap hardwareMap) {
        Arm_1 = hardwareMap.get(DcMotorEx.class, "arm1");
        Arm_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_1.setTargetPosition(0);
        Arm_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                Arm_1.setPower(0.5);
                Arm_2.setPower(0.5);
                initialized = true;
            }

            // checks lift's current position
            double armpos1 = Arm_1.getCurrentPosition();
            double armpos2 = Arm_2.getCurrentPosition();
            packet.put("armPos1", armpos1);
            packet.put("armPos2", armpos2);
            if (armpos1 < 825) {
                return true;
            } else if (armpos2 < 825) {
                return true;
            } else {
                Arm_1.setPower(0);
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
                Arm_1.setPower(-0.5);
                Arm_2.setPower(-0.5);
                initialized = true;
            }

            // checks lift's current position
            double armpos1 = Arm_1.getCurrentPosition();
            double armpos2 = Arm_2.getCurrentPosition();
            packet.put("armPos1", armpos1);
            packet.put("armPos2", armpos2);
            if (armpos1 > 0) {
                return true;
            } else if (armpos2 > 0) {
                return true;
            } else {
                Arm_1.setPower(0);
                Arm_2.setPower(0);
                return false;
            }
        }
    }
    public Action armPar(){
        return new ArmParallel();
    }
}

