package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="fullSendLeft")
public class fullSendLeft extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    private RigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline pipeline;


    private final Pose2d blueHome = new Pose2d(-36, 60, Math.toRadians(270));


    private TrajectorySequence trajectoryToJunction; //check coordinate system in notebook
    private TrajectorySequence trajectoryRecenter;
    private TrajectorySequence trajectoryToParking3;
    private TrajectorySequence trajectoryToParking1;
    private TrajectorySequence goForward;


    private final int initialWaitTime = 250;



    @Override
    public void runOpMode() throws InterruptedException
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        pipeline = new SleevePipeline(telemetry);
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


        Assert.assertNotNull(hardwareMap);

        hardware = new RigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);
        utilities = new Utilities(hardware);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(blueHome);

        turnOnEncoders();


        buildTrajectories();
        utilities.openClaw(false);

        waitForStart();
        if(!opModeIsActive()) {return;}

        utilities.wait(initialWaitTime, telemetry);


        final int identifier = pipeline.getDestination();
        telemetry.addData("Parking", identifier);
        telemetry.update();

        //highJunctionRunPosition();
        drive.followTrajectorySequence(trajectoryToJunction);
        highJunction();
        drive.followTrajectorySequence(trajectoryRecenter); //trajectoryRecenter ends in parking2

        if(identifier == 1)
            drive.followTrajectorySequence(trajectoryToParking1);
//        if (identifier == 2)
//            drive.followTrajectorySequence(trajectoryToParking2); //already here LOLZ
        else if (identifier == 3)
            drive.followTrajectorySequence(trajectoryToParking3);
    }




    public void highJunction()
    {
        utilities.liftArm(1, 4350, telemetry); // .8 5300
        drive.followTrajectorySequence(goForward);
        utilities.lowerArm(1, 400, telemetry); //.8 500
        utilities.openClaw(true);
        utilities.lowerArm(1, 3950, telemetry); //.8 4800

    }
    public void highJunctionRunPosition()
    {
        hardware.liftArm.setTargetPosition(2100);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(goForward);
        hardware.liftArm.setTargetPosition(1900);
        utilities.openClaw(true);
        hardware.liftArm.setTargetPosition(500);
    }


    private void buildTrajectories()
    {
        trajectoryToJunction = drive.trajectorySequenceBuilder(blueHome)
                .forward(48)
                .turn(Math.toRadians(-45))
                .build();
        trajectoryRecenter = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                .back(6.5)
                .turn(Math.toRadians(135))
                .build();
        goForward = drive.trajectorySequenceBuilder(trajectoryToJunction.end()) //trajectoryTo12.end()
                .forward(6.5)
                .build();
        trajectoryToParking1 = drive.trajectorySequenceBuilder(trajectoryRecenter.end()) //beforeJunction goForward.end())
                .forward(22)
                .build();
        trajectoryToParking3 = drive.trajectorySequenceBuilder(trajectoryRecenter.end()) //goForward.end()
                .back(24)
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
