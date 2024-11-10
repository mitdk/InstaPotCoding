/*package org.firstinspires.ftc.teamcode;

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

public class AutoModeRedYellow extends LinearOpMode {
    public class Arm {
        private DcMotorEx Arm;

        public Arm(HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotorEx.class, "arm");
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
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            Intake = hardwareMap.get(CRServo.class, "intake");
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
            Outtake = hardwareMap.get(Servo.class, "outtake");
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
            LinSlideLeft = hardwareMap.get(DcMotorEx.class, "linSlideLeft");
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
            armLinSlide = hardwareMap.get(DcMotorEx.class, "linSlideLeft");
            armLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armLinSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public Action armLinSlideDown() {
            return new AutoModeRedYellow.armLinearSlide.armLinSlideDown();
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
            return new AutoModeRedYellow.armLinearSlide.armLinSlideUp();
        }
        public class armLinSlideDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                }
                double pos = armLinSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1) {

                } else {
                    armLinSlide.setPower(0);
                    return false;
                }
                return false;
            }
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


        TrajectoryActionBuilder RedYellow = drive.actionBuilder(initialPose)
                ///RED SIDE YELLOW SAMPLE
                //Brings the robot to the basket for first sample
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                .afterTime(0.3, linearSlideLeft.linSlideLeftUp())
                //Outtake sample into basket
                .stopAndAdd(outtake.outtakeDispose())
                //Brings robot to second sample block, the middle one of the 3 outside the sub
                .strafeToLinearHeading(new Vector2d(-36.7, -25.8), Math.toRadians(180))
                //Intakes 2nd sample block
                .afterTime(4.5, linearSlideLeft.linSlideLeftDown())
                .afterTime(4.5, outtake.outtakeCollect())
                .afterTime(4.5, arm.armCollect())
                //Intakes 2nd sample block
                .stopAndAdd(intake.intakeCollect())
                //Travel to basket
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .afterTime(7.3, arm.armIntoBasket())
                .afterTime(8.7, intake.intakeDispose())
                .afterTime(9.2, linearSlideLeft.linSlideLeftUp())
                //Deposits 2nd sample block
                .stopAndAdd(outtake.outtakeDispose())
                //goes to 3rd sample
                .strafeToLinearHeading(new Vector2d(-36.7, -25.8), Math.toRadians(180))
                .afterTime(10.5, linearSlideLeft.linSlideLeftDown())
                .afterTime(10.5, outtake.outtakeCollect())
                .lineToY(10)
                .afterTime(12, arm.armCollect())
                //Intakes 3rd sample block
                .stopAndAdd(intake.intakeCollect())
                //Travel to basket
                .lineToY(-10)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .afterTime(14, arm.armIntoBasket())
                .afterTime(15, intake.intakeDispose())
                .afterTime(16, linearSlideLeft.linSlideLeftUp())
                //Deposits 3rd sample block
                .stopAndAdd(outtake.outtakeDispose())
                //Goes to 4th sample block, alliance specific on other side
                .splineToLinearHeading(new Pose2d(36.7,-25.8, Math.toRadians(0)), Math.toRadians(90))
                .afterTime(18.15, linearSlideLeft.linSlideLeftDown())
                .afterTime(18.15, outtake.outtakeCollect())
                .afterTime(20.8, arm.armCollect())
                //Intakes
                .stopAndAdd(intake.intakeCollect())
                //Travel to basket
                .splineToLinearHeading(new Pose2d(-55,-55,Math.toRadians(45)), Math.toRadians(180))
                .afterTime(23.5, arm.armIntoBasket())
                .afterTime(24, intake.intakeDispose())
                .afterTime(25.4, linearSlideLeft.linSlideLeftUp())
                .stopAndAdd(outtake.outtakeDispose())
                //Deposits 4th sample
                .stopAndAdd(outtake.outtakeDispose())
                //Comes to Observation Zone for reentry
                .strafeToLinearHeading(new Vector2d(58,-61),Math.toRadians(90));


        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                            RedYellow.build()
                    )
            );
        }







    }
}

 */