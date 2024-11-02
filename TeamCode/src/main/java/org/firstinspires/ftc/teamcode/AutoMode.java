package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;


@Autonomous(name = "AutoMode")

public class AutoMode extends LinearOpMode {
    public class Arm {
        private DcMotorEx Arm;

        public Arm(HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotorEx.class, "armMotor");
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
    public class Intake{
        private CRServo Intake;

        public Intake(HardwareMap hardwareMap) {
            Intake = hardwareMap.get(CRServo.class, "Intake");
        }
    }
    public class Wrist{
        private Servo Wrist;

        public Wrist(HardwareMap hardwareMap) {
            Wrist = hardwareMap.get(Servo.class, "Servo");
        }
    }
    public class LinearSlideLeft{
        public DcMotorEx LinSlideLeft;

        public LinearSlideLeft(HardwareMap hardwareMap) {
            LinSlideLeft = hardwareMap.get(DcMotorEx.class, "LinearSlideLeft");
            LinSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LinSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class LinSlideLeftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = LinSlideLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1) {

                } else {
                    LinSlideLeft.setPower(0);
                    return false;
                }
                return false;
            }
        }
        public Action LinSlideLeftUp() {
            return new LinSlideLeftUp();
        }
    }
    public class LinearSlideRight {
        public DcMotorEx LinSlideRight;

        public LinearSlideRight(HardwareMap hardwareMap) {
            LinSlideRight = hardwareMap.get(DcMotorEx.class, "LinearSlideLeft");
            LinSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LinSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LinSlideRightUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = LinSlideRight.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1) {

                } else {
                    LinSlideRight.setPower(0);
                    return false;
                }
                return false;
            }
        }
        public Action LinSlideRightUp() {
            return new LinSlideRightUp();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        LinearSlideLeft linearSlideLeft = new LinearSlideLeft(hardwareMap);
        LinearSlideRight linearSlideRight = new LinearSlideRight(hardwareMap);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(48, -40), Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(58, -40), Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(72, -40), Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(66, -66), Math.toRadians(90));








    }
}