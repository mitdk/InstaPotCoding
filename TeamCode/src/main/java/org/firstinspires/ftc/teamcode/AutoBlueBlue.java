package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.jetbrains.annotations.NotNull;


@Autonomous(name = "AutoMode")

public class AutoBlueBlue extends LinearOpMode{
    public class Arm {
        private DcMotorEx Arm_1;
        private DcMotorEx Arm_2;
        public Arm(HardwareMap hardwareMap) {
            Arm_1 = hardwareMap.get(DcMotorEx.class, "arm1");
            Arm_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm_1.setTargetPosition(0);
            Arm_1.setVelocity(2100);
            Arm_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Arm_2 = hardwareMap.get(DcMotorEx.class, "arm2");
            Arm_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm_2.setTargetPosition(0);
            Arm_2.setVelocity(2100);
            Arm_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public class ArmCollect implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm_1.setTargetPosition(100); //Just a placeholder number
                Arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Arm_2.setTargetPosition(100); //Just a placeholder number
                Arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }
        }
        public Action armCollect() {
            return new ArmCollect();
        }

        public class ArmVertical implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm_1.setTargetPosition(0); //Just a placeholder number
                Arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Arm_2.setTargetPosition(0); //Just a placeholder number
                Arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }
        }
        public Action armVertical() {
            return new ArmVertical();
        }

        public class ArmPreSpec implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm_1.setTargetPosition(50); //Just a placeholder number
                Arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Arm_2.setTargetPosition(50); //Just a placeholder number
                Arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }
        }
        public Action armPreSpec() {
            return new ArmPreSpec();
        }

        public class ArmPostSpec implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm_1.setTargetPosition(80); //Just a placeholder number
                Arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Arm_2.setTargetPosition(80); //Just a placeholder number
                Arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }
        }
        public Action armPostSpec() {
            return new ArmPostSpec();
        }

    }

    public class LinSlide {
        public DcMotorEx LinSlide;
        public LinSlide(HardwareMap hardwareMap){
            LinSlide = hardwareMap.get(DcMotorEx.class, "linSlide");
            LinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LinSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LinSlideMove implements Action{
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = LinSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3965) {

                } else {
                    LinSlide.setPower(0);
                    return false;
                }
                return false;
            }
        }
        public Action linSlideUp() {
            return new LinSlideMove();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
