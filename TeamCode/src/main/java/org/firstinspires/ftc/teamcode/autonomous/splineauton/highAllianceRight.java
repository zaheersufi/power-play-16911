package org.firstinspires.ftc.teamcode.autonomous.splineauton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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

@Autonomous(name="highAllianceRight")
public class highAllianceRight extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private UtilitiesUpdated utilities;
    private RigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline pipeline;


    private final Pose2d blueHome = new Pose2d(-36, 66, Math.toRadians(-90));
    private final Pose2d otwJunction = new Pose2d(-15.75, 60, Math.toRadians(-90));
    private final Pose2d inFrontOfJunction = new Pose2d(-13.25, 22, Math.toRadians(0));
    private final Pose2d parkingOne = new Pose2d(-13.25, 12, Math.toRadians(-180));
    private final Pose2d otwTwo = new Pose2d(-20, 13, Math.toRadians(-100));
    private final Pose2d parkingTwo = new Pose2d(-36, 12, Math.toRadians(-180));
    private final Pose2d otw1Three = new Pose2d(-20, 13, Math.toRadians(-150));
    private final Pose2d otw2Three = new Pose2d(-36, 12, Math.toRadians(-180));
    private final Pose2d parkingThree = new Pose2d(-58, 12, Math.toRadians(-180));

    private Trajectory trajectoryToHighAlliance;
    private Trajectory trajectoryGoForward;
    private Trajectory trajectoryToParkingOne;
    private Trajectory trajectoryToParkingTwo;
    private Trajectory trajectoryToParkingThree;
    private Trajectory trajectoryGoBackward;

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

        drive.followTrajectory(trajectoryToHighAlliance);
        highJunction();

        if(identifier == 1)
            drive.followTrajectory(trajectoryToParkingOne);
        else if (identifier == 2)
            drive.followTrajectory(trajectoryToParkingTwo);
        else
            drive.followTrajectory(trajectoryToParkingThree);

    }

    private void buildTrajectories()
    {
        trajectoryToHighAlliance = drive.trajectoryBuilder(blueHome, Math.toRadians(-90))
                .splineToSplineHeading(otwJunction, -60)
                .splineToSplineHeading(inFrontOfJunction, -90)
                .build();
        trajectoryGoForward = drive.trajectoryBuilder(trajectoryToHighAlliance.end(), 0)
                .forward(5)
                .build();
        trajectoryGoBackward = drive.trajectoryBuilder(trajectoryGoForward.end(), 0)
                .back(5)
                .build();
        trajectoryToParkingOne = drive.trajectoryBuilder(trajectoryGoBackward.end(), 0)
                .splineToSplineHeading(parkingOne, -90)
                .build();
        trajectoryToParkingTwo = drive.trajectoryBuilder(trajectoryGoBackward.end(), 0)
                .splineToSplineHeading(otwTwo, -160)
                .splineToSplineHeading(parkingTwo, -180)
                .build();
        trajectoryToParkingThree = drive.trajectoryBuilder(trajectoryGoBackward.end(), 0)
                .splineToSplineHeading(otw1Three, 190)
                .splineToSplineHeading(otw2Three, 180)
                .splineToSplineHeading(parkingThree, 180)
                .build();
        //we have to test if it is tangent or heading where it says startHeading, currently
        //it is the heading and on the Pose2d it also has the heading with the Math.toRadians
    }

    public void highJunction ()
    {
//        utilities.liftArm(1, 4350, telemetry);
//        drive.followTrajectory(trajectoryGoForward);
//        utilities.lowerArm(1, 400, telemetry);
//        utilities.openClaw(true);
//        drive.followTrajectory(trajectoryGoBackward);
//        utilities.lowerArm(1, 3950, telemetry);
        hardware.liftArm.setTargetPosition(2100);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectory(trajectoryGoForward);
        hardware.liftArm.setTargetPosition(1900);
        utilities.openClaw(true);
        hardware.liftArm.setTargetPosition(500);
        drive.followTrajectory(trajectoryGoBackward);


    }

    private void turnOnEncoders()
    {
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
