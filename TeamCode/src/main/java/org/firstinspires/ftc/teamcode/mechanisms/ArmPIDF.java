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
public class ArmPIDF extends OpMode{
    private PIDFController controller;

    public static double p = 0.003, i = 0.001, d = 0.0001;

    public static double f = 0.0001;


    public static int target = 0;

    private final double TICKS_PER_DEGREE = 5281.1/360;

    private DcMotorEx arm1, arm2;


    @Override
    public void init() {
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");


        controller = new PIDFController(p,i,d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);
        int arm1Pos = -arm1.getCurrentPosition();
        int arm2Pos = -arm2.getCurrentPosition();
        double pidf = controller.calculate(arm1Pos, target);
        double pidf2 = controller.calculate(arm2Pos, target);
        double power1 = -pidf;
        double power2 = -pidf2;
        arm1.setPower(power1);
          arm2.setPower(power1);

        telemetry.addData("pos1", arm1Pos);
        telemetry.addData("pos2", arm2Pos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
