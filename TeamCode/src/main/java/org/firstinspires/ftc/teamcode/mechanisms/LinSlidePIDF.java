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
public class LinSlidePIDF extends OpMode{
    public PIDFController controller;

    public static double p = 0.01, i = 0, d = 0.0002;

    public static double f = 0.00001;


    public static int target = 0;

    public final double TICKS_PER_DEGREE = 537.7 /360;

    private DcMotorEx linSlide;


    @Override
    public void init() {
        linSlide = hardwareMap.get(DcMotorEx.class, "armLinSlide");

        controller = new PIDFController(p,i,d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);


        int linSlidePos = -linSlide.getCurrentPosition();
        double pidf = controller.calculate(linSlidePos, target);

        double power1 = -pidf;

        linSlide.setPower(power1);

        telemetry.addData("pos1 ", linSlidePos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
