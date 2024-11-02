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


@Autonomous(name = "AutoMode")

public class AutoMode extends LinearOpMode {
    public class Arm {
        private DcMotorEx Arm;

        public Arm(HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotorEx.class, "armMotor");
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public class ArmCollect implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.setTargetPosition(0);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }
        }
        public class ArmIntoBasket implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.setTargetPosition(1);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }
        }
        public class ArmReset implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.setTargetPosition(2);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }
        }
    }
    public class Intake {
        private CRServo Intake;

        public Intake(HardwareMap hardwareMap) {
            Intake = hardwareMap.get(CRServo.class, "Intake");
        }

        public class IntakeCollect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Intake.setPower(-3);
                return false;
            }
        }

        public class IntakeDispose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Intake.setPower(0.5);
                return false;
            }
        }

        public class IntakeReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Intake.setPower(0.5);
                return false;
            }
        }
    }
    public class Wrist{
        private Servo Wrist;

        public Wrist(HardwareMap hardwareMap) {
            Wrist = hardwareMap.get(Servo.class, "Servo");
        }
        public class WristOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Wrist.setPosition(0.945);
                return false;
            }
        }
        public class WristSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Wrist.setPosition(0.65);
                return false;
            }
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
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                )
        );







    }
    }