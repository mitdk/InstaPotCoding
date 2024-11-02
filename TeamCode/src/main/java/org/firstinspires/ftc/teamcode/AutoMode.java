package org.firstinspires.ftc.teamcode;

// RR-specific imports

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

        import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    public class LinearSlides{
        private DcMotorEx LinSlideLeft;
        private DcMotorEx LinSlideRight;

        public void LinSlideLeft(HardwareMap hardwareMap) {
            LinSlideLeft = hardwareMap.get(DcMotorEx.class, "LinearSlideLeft");
            LinSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LinSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public void LinSlideRight(HardwareMap hardwareMap) {
            LinSlideRight = hardwareMap.get(DcMotorEx.class, "LinearSlideRight");
            LinSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LinSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
