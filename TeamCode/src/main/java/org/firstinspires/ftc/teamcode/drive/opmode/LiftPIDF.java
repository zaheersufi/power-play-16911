package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;



@Config
@Autonomous(group = "drive")
public class LiftPIDF extends LinearOpMode {
    public static int target = 200;

    public static double kP = 10;
    public static double kI = 3;
    public static double kD = 0;
    public static double kF = 0;


    private PIDFCoefficients PIDF = new PIDFCoefficients(kP,kI,kD,kF);

    private NewRigatoniHardware hardware;


    public void runOpMode() throws InterruptedException {
        Assert.assertNotNull(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hardware = new NewRigatoniHardware();
        hardware.initializeSupplementaryMotors(hardwareMap);

        telemetry.addLine("Pos 1"+hardware.liftArm1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addLine("Pos 2"+hardware.liftArm1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addLine("Vel 1"+hardware.liftArm1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addLine("Vel 2"+hardware.liftArm1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));


        hardware.liftArm1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        hardware.liftArm2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

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


        while (!isStopRequested()) {
            hardware.liftArm1.setPower(1.0);
            hardware.liftArm2.setPower(1.0);

            hardware.liftArm1.setTargetPosition(target);
            hardware.liftArm2.setTargetPosition(target);

            telemetry.addData("Target", target);
            telemetry.addData("Lift1 Position", hardware.liftArm1.getCurrentPosition());
            telemetry.addData("Lift2 Position", hardware.liftArm2.getCurrentPosition());
            telemetry.update();


            if (kP != PIDF.p || kD != PIDF.d || kI != PIDF.i || kF != PIDF.f) {
                PIDF = new PIDFCoefficients(kP,kI,kD,kF);

//                hardware.liftArm1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDF);
//                hardware.liftArm2.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDF);

                kP = PIDF.p;
                kI = PIDF.i;
                kD = PIDF.d;
                kF = PIDF.f;
            }

            telemetry.update();
        }


    }


}
