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

public class AutoModeBlueBlue extends LinearOpMode {
    public class Arm {
        private DcMotorEx Arm;

        public Arm(HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotorEx.class, "armMotor");
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm.setTargetPosition(0);
            Arm.setVelocity(2100);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public class ArmCollect implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.setTargetPosition(100);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public Action armCollect() {
            return new ArmCollect();
        }
        public class ArmIntoBasket implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.setTargetPosition(-3400);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public Action armIntoBasket() {
            return new ArmIntoBasket();
        }
        public class ArmReset implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.setTargetPosition(0);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public Action armReset() {
            return new ArmReset();
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
        public Action intakeCollect() {
            return new IntakeCollect();
        }
        public class IntakeDispose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Intake.setPower(3);
                return false;
            }
        }
        public Action intakeDispose() {
            return new IntakeDispose();
        }
        public class IntakeReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Intake.setPower(0);
                return false;
            }
        }
        public Action intakeReset() {
            return new IntakeReset();
        }
    }
    public class Outtake {
        private Servo Outtake;

        public Outtake(HardwareMap hardwareMap) {
            Outtake = hardwareMap.get(Servo.class, "Outtake");
        }

        public class OuttakeCollect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Outtake.setPosition(0.9);
                return false;
            }
        }
        public Action outtakeCollect() {
            return new OuttakeCollect();
        }

        public class OuttakeDispose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Outtake.setPosition(0.65);
                return false;
            }
        }
        public Action outtakeDispose() {
            return new OuttakeDispose();
        }
    }
    public class LinearSlideLeft{
        public DcMotorEx LinSlideLeft;

        public LinearSlideLeft(HardwareMap hardwareMap) {
            LinSlideLeft = hardwareMap.get(DcMotorEx.class, "LinearSlideLeft");
            LinSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LinSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public Action linSlideLeftDown() {
            return new LinSlideLeftDown();
        }

        public class LinSlideLeftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = LinSlideLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                while (pos < 1000) {
                    LinSlideLeft.setPower(100);
                }
                if (pos == 1000) {
                    LinSlideLeft.setPower(0);
                    return false;
                }
                return false;
            }
        }
        public Action linSlideLeftUp() {
            return new LinSlideLeftUp();
        }
        public class LinSlideLeftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = LinSlideLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1) {

                } else {
                    LinSlideLeft.setPower(0);
                    return false;
                }
                return false;
            }
        }
    }
    public class LinearSlideLeft {
        public DcMotorEx LinSlideLeft;

        public LinearSlideLeft(HardwareMap hardwareMap) {
            LinSlideLeft = hardwareMap.get(DcMotorEx.class, "LinearSlideLeft");
            LinSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LinSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public Action linSlideLefRetract() {            return new LinSlideLeftDown();
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
        public Action linSlideLeftUp() {
            return new LinSlideLeftUp();
        }
        public class LinSlideLeftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = LinSlideLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                while (pos > 1000) {
                    LinSlideLeft.setPower(-100);
                }
                if (pos == 1000) {
                    LinSlideLeft.setPower(0);
                    return false;
                }
                return false;
            }
        }
    }
    public class armLinearSlide{
        public DcMotorEx armLinSlide;

        public armLinearSlide(HardwareMap hardwareMap) {
            armLinSlide = hardwareMap.get(DcMotorEx.class, "ArmLinearSlide");
            armLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armLinSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class armLinSlideUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = armLinSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1) {

                } else {
                    armLinSlide.setPower(0);
                    return false;
                }
                return false;
            }
        }
        public Action armlinSlideUp() {
            return new armLinSlideUp();
        }
        public class armLinSlideDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = armLinSlide.getCurrentPosition();
                packet.put("armLinSlidePos", pos);
                if (pos > 1) {

                } else {
                    armLinSlide.setPower(0);
                    return false;
                }
                return false;
            }
        }
        public Action armLinSlideDown() {
            return new armLinSlideDown();
        }
        public Action armLinSlideStraight() {
            return new armLinSlideStraight();
        }
        public class armLinSlideStraight implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = armLinSlide.getCurrentPosition();
                packet.put("armLinSlidePos", pos);
                if (pos > 1) {

                } else {
                    armLinSlide.setPower(0);
                    return false;
                }
                return false;
            }

        }
        public Action ArmLinSlideStraight() {
            return new armLinSlideStraight();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        LinearSlideLeft linearSlideLeft = new LinearSlideLeft(hardwareMap);
        armLinearSlide armLinSlide = new armLinearSlide(hardwareMap);

        TrajectoryActionBuilder BlueBlue = drive.actionBuilder(initialPose)
                //BLUE SIDE BLUE SAMPLE
                //Brings the robot to the basket for first sample
                .splineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-135)), Math.toRadians(0)).afterTime(1, linearSlideLeft.linSlideLeftUp())
                //Disposes 1st sample into basket
                .stopAndAdd(outtake.outtakeDispose())
                //Brings robot to second sample block, the middle one of the 3 outside the sub
                .afterTime(4, outtake.outtakeCollect())
                .afterTime(4, linearSlideLeft.linSlideLeftDown())
                .splineToLinearHeading(new Pose2d(-36.7,25.8, Math.toRadians(180)), Math.toRadians(-90))
                .afterTime(6, arm.armCollect())
                .afterTime(6, armLinSlide.armlinSlideUp())
                //Intakes 2nd sample block
                .stopAndAdd(intake.intakeCollect())
                //Travel to basket
                .afterTime(8, armLinSlide.armLinSlideDown())
                .afterTime(9, arm.armIntoBasket())
                .afterTime(10, armLinSlide.armlinSlideUp())
                .strafeToConstantHeading(new Vector2d(-36.7, 40.8))
                .afterTime(10.5, intake.intakeDispose())
                .afterTime(11, linearSlideLeft.linSlideLeftUp())
                .splineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-135)), Math.toRadians(0))
                //Deposits 2nd sample block
                .stopAndAdd(outtake.outtakeDispose())
                .afterTime(14, outtake.outtakeCollect())
                //goes to 3rd sample
                .afterTime(14, linearSlideLeft.linSlideLeftDown())
                .splineToLinearHeading(new Pose2d(-48,25.8, Math.toRadians(180)), Math.toRadians(-90))
                .afterTime(17, arm.armCollect())
                .afterTime(17, armLinSlide.armlinSlideUp())
                //Intakes 3rd sample block
                .stopAndAdd(intake.intakeCollect())
                //Travel to Basket
                .afterTime(19, armLinSlide.armLinSlideDown())
                .afterTime(20, arm.armIntoBasket())

                .afterTime(21, intake.intakeDispose())
                .afterTime(23, linearSlideLeft.linSlideLeftUp())
                .strafeToConstantHeading(new Vector2d(-48, 40.8))
                .splineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-135)), Math.toRadians(0))
                .stopAndAdd(outtake.outtakeDispose())

                .afterTime(26, outtake.outtakeCollect())
                .afterTime(23, linearSlideLeft.linSlideLeftDown())
                //Travel to park zone
                .strafeToLinearHeading(new Vector2d(-58, 61), Math.toRadians(180));
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                            BlueBlue.build()
                    )
            );
        }







    }
}