package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Left")
public class Left extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    private RigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private HsvMaskPipeline pipeline;


    private final Pose2d blueHome = new Pose2d(-36, -60, Math.toRadians(90));
    private final Pose2d beforeJunction = new Pose2d(-15, -24, Math.toRadians(0));


    private TrajectorySequence trajectoryTo12; //check coordinate system in notebook
    private TrajectorySequence trajectoryToParking3;
    private TrajectorySequence trajectoryToParking2;
    private TrajectorySequence trajectoryToParking1;
    private TrajectorySequence goForward;


    private final int initialWaitTime = 0;



    @Override
    public void runOpMode()
    {
        Assert.assertNotNull(hardwareMap);

        hardware = new RigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);


        turnOnEncoders();
        utilities = new Utilities(hardware);


        pipeline = new HsvMaskPipeline(telemetry);
        setUpCamera();


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(blueHome);
        buildTrajectories();

        waitForStart();
        if(!opModeIsActive()) {return;}

        utilities.openClaw(false);
        utilities.wait(initialWaitTime, telemetry);

        int identifier = pipeline.getDestination();

        telemetry.addData("Parking", identifier);
        telemetry.update();


        drive.followTrajectorySequence(trajectoryTo12);
        highJunction();

        if(identifier == 1)
            drive.followTrajectorySequence(trajectoryToParking1);
        else if (identifier == 2)
            drive.followTrajectorySequence(trajectoryToParking2);
        else
            drive.followTrajectorySequence(trajectoryToParking3);

    }




    public void highJunction ()
    {
        utilities.liftArm(.8, 5300, telemetry);
        drive.followTrajectorySequence(goForward);
        utilities.lowerArm(.8, 500, telemetry);
        utilities.openClaw(true);
        utilities.lowerArm(.8, 4800, telemetry);

    }



    private void buildTrajectories()
    {
        trajectoryTo12 = drive.trajectorySequenceBuilder(blueHome)
                .forward(6)
                .turn(Math.toRadians(-90))
                .forward(21)
                .strafeLeft(37)
                .build();

        goForward = drive.trajectorySequenceBuilder(trajectoryTo12.end()) //trajectoryTo12.end()
                .forward(6)
                .build();

        trajectoryToParking1 = drive.trajectorySequenceBuilder(goForward.end()) //goForward.end()
                .strafeLeft(12)
                .turn(Math.toRadians(180))
                .forward(44)
                .strafeLeft(1)
                .build();

        trajectoryToParking2 = drive.trajectorySequenceBuilder(goForward.end()) //goForward.end()
                .strafeLeft(14)
                .turn(Math.toRadians(180))
                .forward(22)
                .build();

        trajectoryToParking3 = drive.trajectorySequenceBuilder(goForward.end()) //beforeJunction goForward.end())
                .strafeLeft(14)
                .turn(Math.toRadians(180))
                .back(2)
                .build();

    }



    private void turnOnEncoders()
    {
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



    public void setUpCamera()
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
                telemetry.addData("ERROR", "*Camera could not be opened*");
                telemetry.update();
            }
        });

    }
}
