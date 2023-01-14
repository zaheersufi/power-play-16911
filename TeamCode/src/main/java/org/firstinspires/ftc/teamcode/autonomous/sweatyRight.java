package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name="sweatyRight")
public class sweatyRight extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    private RigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline pipeline;


    private final Pose2d blueHome = new Pose2d(-36, 60, Math.toRadians(270));


    private TrajectorySequence trajectoryTo12; //check coordinate system in notebook
    private TrajectorySequence trajectoryToParking3;
    private TrajectorySequence trajectoryToParking2;
    private TrajectorySequence adjustToParkingCenter;
    private TrajectorySequence goForward;


    private final int initialWaitTime = 250;



    @Override
    public void runOpMode() throws InterruptedException
    {
        pipeline = new SleevePipeline(telemetry);
        setUpCamera(pipeline);


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
        drive.followTrajectorySequence(trajectoryTo12);
        highJunction();
        drive.followTrajectorySequence(adjustToParkingCenter);

//        if(identifier == 1)
//             // drive.followTrajectorySequence(trajectoryToParking1); ITS ALREADY IN THAT PARKING LOL
        if (identifier == 2)
            drive.followTrajectorySequence(trajectoryToParking2);
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
        trajectoryTo12 = drive.trajectorySequenceBuilder(blueHome)
//                .forward(6)
//                .turn(Math.toRadians(90))
//                .forward(24.25)
//                .turn(Math.toRadians(-90))
//                .forward(50)
                .strafeLeft(24.25)
                .forward(56)
                .turn(Math.toRadians(-65))
                .build();

        goForward = drive.trajectorySequenceBuilder(trajectoryTo12.end()) //trajectoryTo12.end()
                .forward(5)
                .build();

        trajectoryToParking3 = drive.trajectorySequenceBuilder(goForward.end()) //beforeJunction goForward.end())
                .forward(46)
                .build();

        trajectoryToParking2 = drive.trajectorySequenceBuilder(goForward.end()) //goForward.end()
                .forward(22)
                .build();

        adjustToParkingCenter = drive.trajectorySequenceBuilder(goForward.end())
                .back(8.5)
                .turn(Math.toRadians(-25))
                .forward(3)
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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("ERROR", "*Camera could not be opened* "+errorCode);
                telemetry.update();
            }
        });

    }
}
