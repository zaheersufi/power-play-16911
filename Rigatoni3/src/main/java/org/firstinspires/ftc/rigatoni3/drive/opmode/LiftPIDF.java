package org.firstinspires.ftc.rigatoni3.drive.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.rigatoni3.hardware.NewRigatoniHardware;



/**
 * A LinearOpMode that implements a PIDF control loop for the linear slide/lift of a robot.
 *
 * The file implements a PID (Proportional-Integral-Derivative) control loop to control
 * the robot's two lift motors. The goal of this control loop is to achieve a stable and
 * smooth control of the lift by constantly adjusting the motors' speed based on the error
 * between the target position and the current lift position (given by encoders)
 *
 * @author Rigatoni Pastabots, 16911
 */
@Config
@Autonomous(group = "drive")
public class LiftPIDF extends LinearOpMode {
    // The target position for the linear slide/lift in encoder ticks.
    public static int target = 200;

    public static double kP = 10;   // The proportional gain (P) of the PIDF control loop.
    public static double kI = 2;    // The integral gain (I) of the PIDF control loop.
    public static double kD = 0;    // The derivative gain (D) of the PIDF control loop.
    public static double kF = 75;   // The feedforward gain (F) of the PIDF control loop.


    /**
     * The {@link PIDFCoefficients} that are used to initialize the control loop.
     */
    private PIDFCoefficients PIDF = new PIDFCoefficients(10,2,0,75);

    private NewRigatoniHardware hardware;



    /**
     * The main loop of the opmode, containing the PIDF control loop for the linear slide/lift.
     *
     * @throws InterruptedException if the opmode is interrupted
     */
    public void runOpMode() throws InterruptedException {
        // Assert that the hardwareMap is not null
        Assert.assertNotNull(hardwareMap);

        // Initialize the dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware object
        hardware = new NewRigatoniHardware();
        hardware.initializeSupplementaryMotors(hardwareMap);

        // Set the PIDF coefficients for the lift arms
        hardware.liftArm1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        hardware.liftArm2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        // Set the zero power behavior for the lift arms to float
        hardware.liftArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.liftArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset the encoders for the lift arms
        hardware.liftArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.liftArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for the lift arms
        hardware.liftArm1.setTargetPosition(target);
        hardware.liftArm2.setTargetPosition(target);

        // Set the run mode for the lift arms to run to position
        hardware.liftArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.liftArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Add a telemetry message indicating that the opmode is ready
        telemetry.addLine("Ready!");
        telemetry.update();

        // Wait for the start of the opmode
        waitForStart();
        if (isStopRequested()) return;

        // Continuously execute the following code while the opmode is running
        while (!isStopRequested()) {
            // Sets the power of the lift arm motors to 1.0
            hardware.liftArm1.setPower(1.0);
            hardware.liftArm2.setPower(1.0);

            // Sets the target position of the lift arm motors to the value of the `target` variable.
            hardware.liftArm1.setTargetPosition(target);
            hardware.liftArm2.setTargetPosition(target);

            // Adds telemetry data to the driver station display, showing the values of the
            // `target` variable, and the current position of each lift arm motor.
            telemetry.addData("Target", target);
            telemetry.addData("Lift1 Position", hardware.liftArm1.getCurrentPosition());
            telemetry.addData("Lift2 Position", hardware.liftArm2.getCurrentPosition());
            telemetry.update();


            // If the values of the `kP`, `kD`, `kI`, or `kF` variables have changed,
            // update the PIDF coefficients of the lift arm motors to the new values.
            if (kP != PIDF.p || kD != PIDF.d || kI != PIDF.i || kF != PIDF.f) {
                PIDF = new PIDFCoefficients(kP,kI,kD,kF);

                hardware.liftArm1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
                hardware.liftArm2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

                kP = PIDF.p;
                kI = PIDF.i;
                kD = PIDF.d;
                kF = PIDF.f;
            }

            // Updates the telemetry data displayed on the driver station.
            telemetry.update();
        }


    }


}
