package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo wrist;
    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
    }
    public class WristIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0.65);
            return false;
        }
    }
    public Action wristIntake() {
        return new WristIntake();
    }
    public class WristSpecimen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0);
            return false;
        }
    }
    public Action wristSpecimen() {
        return new WristSpecimen();
    }
    public class WristFlatout implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0.525);
            return false;
        }
    }
    public Action wristFlatout() {
        return new WristFlatout();
    }
}