package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;



@Config
@TeleOp(name="LiftPIDF")
public class LiftPIDF extends LinearOpMode {
    public static int target = 200;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;


    private PIDFCoefficients PIDF = new PIDFCoefficients(kP,kI,kD,kF);

    private NewRigatoniHardware hardware;


    public void runOpMode() {
        Assert.assertNotNull(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hardware = new NewRigatoniHardware();
        hardware.initializeSupplementaryMotors(hardwareMap);


        hardware.liftArm1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDF);
        hardware.liftArm2.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDF);

        hardware.liftArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.liftArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hardware.liftArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.liftArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.liftArm1.setTargetPosition(target);
        hardware.liftArm2.setTargetPosition(target);

        hardware.liftArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.liftArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;


        double lastKp = PIDF.p;
        double lastKi = PIDF.i;
        double lastKd = PIDF.d;
        double lastKf = PIDF.f;


        while (!isStopRequested()) {
            hardware.liftArm1.setPower(1.0);
            hardware.liftArm2.setPower(1.0);

            telemetry.addData("Target:", target);
            telemetry.addData("Lift1 Position:", hardware.liftArm1.getCurrentPosition());
            telemetry.addData("Lift2 Position:", hardware.liftArm2.getCurrentPosition());
            telemetry.update();


            if (lastKp != PIDF.p || lastKd != PIDF.d || lastKi != PIDF.i || lastKf != PIDF.f) {
                PIDF = new PIDFCoefficients(kP,kI,kD,kF);

                hardware.liftArm1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDF);
                hardware.liftArm2.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDF);

                lastKp = PIDF.p;
                lastKi = PIDF.i;
                lastKd = PIDF.d;
                lastKf = PIDF.f;
            }

            telemetry.update();
        }


    }


}
