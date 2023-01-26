package org.firstinspires.ftc.teamcode.autonomous.current;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.autonomous.NewUtilities;
import org.firstinspires.ftc.teamcode.autonomous.Utilities;
import org.firstinspires.ftc.teamcode.autonomous.pipelines.SleevePipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;
import org.firstinspires.ftc.teamcode.hardware.OldRigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;



@SuppressWarnings({"unused"})
public abstract class genericAuton extends LinearOpMode
{
    public SampleMecanumDrive drive;
    public NewUtilities utilities;
    public NewRigatoniHardware hardware;
    public OpenCvInternalCamera webcam;
    public SleevePipeline sleevePipeline;

    public int identifier;



    /**
     * This method sets everything up for autonomous: it initializes components
     * and variables.
     *
     * @throws  InterruptedException    in case the thread is interrupted
     */
    @Override
    public void runOpMode() throws InterruptedException
    {
        sleevePipeline = new SleevePipeline(telemetry);
        setUpCamera(sleevePipeline);


        Assert.assertNotNull(hardwareMap);
        hardware = new NewRigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);
        utilities = new NewUtilities(hardware);
        drive = new SampleMecanumDrive(hardwareMap);

        buildTrajectories();
        utilities.openClaw(false);

        telemetry.addData("Init Parking", sleevePipeline.getDestination());
        telemetry.update();

        waitForStart();
        if(!opModeIsActive()) {return;}
        utilities.wait(250, telemetry);


        identifier = sleevePipeline.getDestination();
        telemetry.addData("Parking", identifier);
        telemetry.update();


        try {
            run();
        } catch (Throwable t) {
            hardware.robotStopAllMotion();

            // Expected due to timer expiration or "Stop" button pressed.
            if (t instanceof NewRigatoniHardware.StopImmediatelyException) {
                telemetry.addData("Stop Requested", "");
                telemetry.update();
                return;
            }

            telemetry.addData("Exception caught!", t);
            telemetry.update();

            if (t instanceof RuntimeException) {
                throw (RuntimeException) t;
            }

            throw new RuntimeException(t);
        }

    }



    /**
     * This method is to be called in the runOpMode, and it specifies all
     * the actions ran during autonomous.
     */
    public abstract void run();



    /**
     * This method is to be called in the runOpMode, and it sets up all trajectories
     * that will be used by the robot.
     *
     * Defines and builds the different trajectories used.
     */
    public abstract void buildTrajectories();



    /**
     * Sets up the camera/webcam and reading box which scans in the parking zone
     * through the use of color on our custom sleeve’s symbols.
     */
    public void setUpCamera(SleevePipeline pipeline)
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("ERROR", "*Camera could not be opened* "+errorCode);
                telemetry.update();
            }
        });

    }



}
