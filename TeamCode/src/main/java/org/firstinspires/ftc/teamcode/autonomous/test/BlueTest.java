package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.autonomous.NewUtilities;
import org.firstinspires.ftc.teamcode.autonomous.pipelines.BlueStackPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Disabled
@Autonomous(name="BlueTest")
public class BlueTest extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private NewUtilities utilities;
    private NewRigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private BlueStackPipeline stackPipeline;

    private TrajectorySequence stackCorrection;


    private final Pose2d HOME = new Pose2d(0, 0, Math.toRadians(0));



    /**
     * Reads the parking position, scores a cone in the
     * high junction, and parks in the space determined
     * by the custom sleeve.
     *
     * @throws  InterruptedException    in case the thread is interrupted
     */
    @Override
    public void runOpMode() throws InterruptedException
    {
        stackPipeline = new BlueStackPipeline(telemetry);
        setUpCamera();
        webcam.setPipeline(stackPipeline);


        Assert.assertNotNull(hardwareMap);
        hardware = new NewRigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        utilities = new NewUtilities(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(HOME);
        hardware.turnOnDriveEncoders();


        waitForStart();
        if(!opModeIsActive()) return;


        utilities.wait(100, telemetry);
        double displacement = stackPipeline.getDisplacement(40);
        telemetry.addData("Displacement", displacement);
        telemetry.update();


        if (displacement > 1) {
            stackCorrection = drive.trajectorySequenceBuilder(HOME)
                    .strafeRight(Math.abs(displacement)).build();
        } else if (displacement < -1) {
            stackCorrection = drive.trajectorySequenceBuilder(HOME)
                    .strafeLeft(Math.abs(displacement)).build();
        } else {
            stackCorrection = drive.trajectorySequenceBuilder(HOME)
                    .forward(0).build();
        }

        drive.followTrajectorySequence(stackCorrection);
    }


    /**
     * Set up the webcam in an inverted horizontal position
     */
    public void setUpCamera()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
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
