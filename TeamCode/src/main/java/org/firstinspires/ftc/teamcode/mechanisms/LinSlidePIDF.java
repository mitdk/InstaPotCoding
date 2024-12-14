package org.firstinspires.ftc.teamcode.mechanisms;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class LinSlidePIDF extends OpMode{
    private PIDFController controller;

    public static double p = 0, i = 0, d = 0;

    public static double f = 0;


    public static int target = 0;

    private final double TICKS_PER_DEGREE = 537.6/360;

    private DcMotorEx linSlide;


    @Override
    public void init() {
        controller = new PIDFController(p,i,d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linSlide = hardwareMap.get(DcMotorEx.class, "linSlide");
    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);


        int linSlidePos = linSlide.getCurrentPosition();
        double pidf = controller.calculate(linSlidePos, target);

        double power1 = pidf;

        linSlide.setPower(power1);

        telemetry.addData("pos1 ", linSlidePos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
