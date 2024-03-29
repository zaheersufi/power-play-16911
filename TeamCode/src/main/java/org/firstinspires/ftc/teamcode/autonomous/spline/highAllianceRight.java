package org.firstinspires.ftc.teamcode.autonomous.spline;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.autonomous.Utilities;
import org.firstinspires.ftc.teamcode.autonomous.pipelines.SleevePipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.OldRigatoniHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Disabled
@Autonomous(name="highAllianceRight")
public class highAllianceRight extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    private OldRigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline pipeline;


    private final Pose2d rightHome = new Pose2d(36.0, -62.5, Math.toRadians(90.0));

    private final Pose2d otwJunction = new Pose2d(10.0, -45.0, Math.toRadians(90.0));
    private final Pose2d inFrontOfJunction = new Pose2d(8.0, -24.0, Math.toRadians(180.0));

    private final Pose2d parkingOne = new Pose2d(13.25, -12.0, Math.toRadians(0.0));

    private final Pose2d otwTwo = new Pose2d(20.0, -12.0, Math.toRadians(90.0));
    private final Pose2d parkingTwo = new Pose2d(36.0, -12.0, Math.toRadians(0.0));

    private final Pose2d otw1Three = new Pose2d(20.0, -13.0, Math.toRadians(70.0));
    private final Pose2d otw2Three = new Pose2d(36.0, -12.0, Math.toRadians(0.0));
    private final Pose2d parkingThree = new Pose2d(58.5, -12.0, Math.toRadians(0.0));


    private Trajectory trajectoryToJunction;
    private Trajectory trajectoryGoForward;
    private Trajectory trajectoryGoBackward;
    private Trajectory trajectoryToParkingOne;
    private Trajectory trajectoryToParkingTwo;
    private Trajectory trajectoryToParkingThree;

    private final int initialWaitTime = 250;



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
        pipeline = new SleevePipeline(telemetry, 130, 110);
        setUpCamera(pipeline);

        Assert.assertNotNull(hardwareMap);

        hardware = new OldRigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);
        utilities = new Utilities(hardware);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(rightHome);

        turnOnEncoders();


        buildTrajectories();
        utilities.openClaw(false);

        waitForStart();
        if(!opModeIsActive()) {return;}

        utilities.wait(initialWaitTime, telemetry);


        final int identifier = pipeline.getDestination();
        telemetry.addData("Parking", identifier);
        telemetry.update();

        drive.followTrajectory(trajectoryToJunction);
        highJunction();

        if(identifier == 1)
            drive.followTrajectory(trajectoryToParkingOne);
        else if (identifier == 2)
            drive.followTrajectory(trajectoryToParkingTwo);
        else
            drive.followTrajectory(trajectoryToParkingThree);

    }


    /**
     * Defines and builds the different trajectories used.
     */
    private void buildTrajectories()
    {
        trajectoryToJunction = drive.trajectoryBuilder(rightHome, Math.toRadians(180.0))
                .splineToSplineHeading(otwJunction, Math.toRadians(90.0))
                .splineToSplineHeading(inFrontOfJunction, Math.toRadians(120.0))
                .build();
        trajectoryGoForward = drive.trajectoryBuilder(trajectoryToJunction.end(), trajectoryToJunction.end().getHeading())
                .forward(4.0)
                .build();
        trajectoryGoBackward = drive.trajectoryBuilder(trajectoryGoForward.end(), trajectoryGoForward.end().getHeading())
                .back(6.0)
                .build();
        trajectoryToParkingOne = drive.trajectoryBuilder(trajectoryGoBackward.end(), Math.toRadians(90.0))
                .splineToSplineHeading(parkingOne, Math.toRadians(90.0))
                .build();
        trajectoryToParkingTwo = drive.trajectoryBuilder(trajectoryGoBackward.end(), Math.toRadians(90.0))
                .splineToSplineHeading(otwTwo, Math.toRadians(10.0))
                .splineToSplineHeading(parkingTwo, Math.toRadians(0.0))
                .build();
        trajectoryToParkingThree = drive.trajectoryBuilder(trajectoryGoBackward.end(), Math.toRadians(90.0))
                .splineToSplineHeading(otw1Three, Math.toRadians(10.0))
                .splineToSplineHeading(otw2Three, Math.toRadians(0.0))
                .splineToSplineHeading(parkingThree, Math.toRadians(0.0))
                .build();

    }


    /**
     * When the robot is in front of the High Junction, it
     * lifts the arm, approaches it, reads the position of the
     * junction, corrects its position to align with the pole,
     * lets the cone go (opens claw), and lowers the lift back down.
     */
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


    /**
     * Turns on the encoders of all drive motors and the lift motor.
     */
    private void turnOnEncoders()
    {
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Set up the webcam in an inverted horizontal position
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
