package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private RobotVision robotVision;
    RigatoniHardware hardware;

    private int initialWaitTime = 0;

    private final Pose2d blueHome = new Pose2d(-36, 60, Math.toRadians(270));
    private final Pose2d beforeJunction = new Pose2d(-15, 24, Math.toRadians(0));

    private TrajectorySequence trajectoryTo12; //check coordinate system in notebook
    private TrajectorySequence trajectoryToParking3;
    private TrajectorySequence trajectoryToParking2;
    private TrajectorySequence trajectoryToParking1;
    private TrajectorySequence goForward;

    OpenCvInternalCamera webcam;
    PowerPlayPipeline_HSV pipeline;

    @Override
    public void runOpMode()
    {
        hardware = new RigatoniHardware(); //Horizontal Claw
        Assert.assertNotNull(hardwareMap);
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);

        turnOnEncoders(hardware);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new PowerPlayPipeline_HSV(telemetry);
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "*Camera could not be opened*");
                telemetry.update();
            }
        });


        int identifier = pipeline.getDestination();
        utilities = new Utilities(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(blueHome);
        buildTrajectories();

        waitForStart();
        if(!opModeIsActive()) {return;}

        utilities.openClaw(false);
        utilities.wait(initialWaitTime, telemetry);

        drive.followTrajectorySequence(trajectoryTo12);
        highJunction(telemetry, drive);
        if(identifier==0)
            drive.followTrajectorySequence(trajectoryToParking1);
        else if (identifier == 1)
            drive.followTrajectorySequence(trajectoryToParking2);
        else
            drive.followTrajectorySequence(trajectoryToParking3);

    }
    private void turnOnEncoders(RigatoniHardware hardware)
    {
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveForward()
    {
        drive.followTrajectorySequence(goForward);
    }
    public void highJunction (Telemetry telemetry, SampleMecanumDrive drive)
    {
        //dropCone(.8, 5400, telemetry, drive);
        utilities.liftArm(.8, 5400, telemetry, drive);
        moveForward();
        utilities.openClaw(true);
        utilities.lowerArm(.8, 5400, telemetry, drive);
    }
    private void buildTrajectories()
    {
        trajectoryTo12 = drive.trajectorySequenceBuilder(blueHome)
                .forward(6)
                .turn(Math.toRadians(-90))
                .forward(21)
                .strafeLeft(37)
                .build();
        trajectoryToParking3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeLeft(14)
                .turn(Math.toRadians(180))
                .forward(47)
                .build();
        trajectoryToParking2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeLeft(14)
                .turn(Math.toRadians(180))
                .forward(24)
                .build();
        trajectoryToParking1 = drive.trajectorySequenceBuilder(beforeJunction)
                .strafeLeft(14)
                .turn(Math.toRadians(180))
                .back(4)
                .build();
        goForward = drive.trajectorySequenceBuilder(beforeJunction)
                .forward(4.5)
                .build();
    }
}
