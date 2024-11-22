package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinSlide {
    public DcMotorEx LinSlide;
    public LinSlide(HardwareMap hardwareMap){
        LinSlide = hardwareMap.get(DcMotorEx.class, "armLinSlide");
        LinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinSlide.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public class LinSlideUp implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                LinSlide.setPower(0.4);
                initialized = true;
            }
            double pos = LinSlide.getCurrentPosition();
            packet.put("linSlidePos", pos);
            if (pos < 3965) {
                return true;
            } else {
                LinSlide.setPower(0);
                return false;
            }
        }
    }
    public Action linSlideUp() {
        return new LinSlideUp();
    }
    public class LinSlideDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                LinSlide.setPower(-0.4);
                initialized = true;
            }
            double pos = LinSlide.getCurrentPosition();
            packet.put("linSlidePos", pos);
            if (pos > 0) {
                return true;
            } else {
                LinSlide.setPower(0);
                return false;
            }
        }
    }
    public Action linSlideDown() {
        return new LinSlideDown();
    }
}