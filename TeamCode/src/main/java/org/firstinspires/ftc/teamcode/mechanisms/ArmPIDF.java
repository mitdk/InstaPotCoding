package org.firstinspires.ftc.teamcode.mechanisms;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class ArmPIDF extends OpMode{
    private PIDFController controller;

    public static double p = 0, i = 0, d = 0;

    public static double f = 0;

    public static double targetPos = 1400;

    private final double TICKS_PER_DEGREE = 1425 / 90;

    private DcMotorEx arm;


    @Override
    public void init() {
        controller = new PIDFController(p,i,d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "arm");
    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);

        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, targetPos);
        double ff = Math.cos(Math.toRadians(targetPos / TICKS_PER_DEGREE));
        f = ff;

        double power = pid + ff;

        arm.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", targetPos);
        telemetry.update();
    }
}
