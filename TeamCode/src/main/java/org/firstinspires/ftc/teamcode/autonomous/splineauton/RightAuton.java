package org.firstinspires.ftc.teamcode.autonomous.splineauton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.autonomous.SleevePipeline;
import org.firstinspires.ftc.teamcode.autonomous.Utilities;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="RightAuton")
public class RightAuton extends LinearOpMode {
    private SampleMecanumDrive drive;
    private UtilitiesUpdated utilities;
    private RigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline pipeline;


    private final Pose2d blueHome = new Pose2d(-36, 60, Math.toRadians(270));
    private final Pose2d forwardTurn = new Pose2d(-15.75, 60, Math.toRadians(-90));
    private final Pose2d infrontOfJunction = new Pose2d(-14.25, 22, Math.toRadians(0));
    private final Pose2d moveForward = new Pose2d(-11.25, 22, Math.toRadians(0));
    //doesn't go to moveForward *****


    private Trajectory trajectoryTo12; //check coordinate system in notebook
    private TrajectorySequence trajectoryToParking3;
    private TrajectorySequence trajectoryToParking2;
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
        utilities = new UtilitiesUpdated(hardware);

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

        //highJunction(); commented out to test path before testing the rest
        drive.followTrajectory(trajectoryTo12);


        if(identifier == 1)
            drive.followTrajectorySequence(trajectoryToParking1);
        else if (identifier == 2)
            drive.followTrajectorySequence(trajectoryToParking2);
        else
            drive.followTrajectorySequence(trajectoryToParking3);

    }




    public void highJunction ()
    {
//        utilities.liftArm(1, 4350, telemetry); // .8 5300
//        //drive.followTrajectorySequence(goForward);
//        utilities.lowerArm(1, 400, telemetry); //.8 500
//        utilities.openClaw(true);
//        utilities.lowerArm(1, 3950, telemetry); //.8 4800
        hardware.liftArm.setTargetPosition(4200);
        utilities.openClaw(true);
        hardware.liftArm.setTargetPosition(0);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //hardware.liftArm.setPower(1)




    }



    private void buildTrajectories()
    {
        trajectoryTo12 = drive.trajectoryBuilder(blueHome, -10)
                .splineToSplineHeading(forwardTurn, Math.toRadians(-60))
                .splineToSplineHeading(infrontOfJunction, Math.toRadians(-90))
                //,splineToSplineHeading(moveForward,
                .build();


        trajectoryToParking3 = drive.trajectorySequenceBuilder(goForward.end()) //beforeJunction goForward.end())
                .strafeRight(12)
                .turn(Math.toRadians(180))
                .build();

        trajectoryToParking2 = drive.trajectorySequenceBuilder(goForward.end()) //goForward.end()
                .strafeRight(12)
                .turn(Math.toRadians(180))
                .forward(22)
                .build();

        trajectoryToParking1 = drive.trajectorySequenceBuilder(goForward.end()) //goForward.end()
                .strafeRight(12)
                .turn(Math.toRadians(180))
                .forward(44.5)
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
