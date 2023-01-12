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



@Autonomous(name="fullSendMidRight")
public class fullSendMidRight extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    private RigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline sleevePipeline;


    private TrajectorySequence trajectoryToJunction;
    private TrajectorySequence goForward;
    private TrajectorySequence trajectoryRecenter;
    private TrajectorySequence trajectoryToParking1;
    private TrajectorySequence trajectoryToParking3;


    private final Pose2d HOME = new Pose2d(-36, 60, Math.toRadians(270));


    private final int WAIT_TIME = 250;



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
        sleevePipeline = new SleevePipeline(telemetry);
        setUpCamera(sleevePipeline);


        Assert.assertNotNull(hardwareMap);
        hardware = new RigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);
        utilities = new Utilities(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(HOME);


        turnOnEncoders();
        buildTrajectories();
        utilities.openClaw(false);
        waitForStart();
        if(!opModeIsActive()) {return;}
        utilities.wait(WAIT_TIME, telemetry);


        final int IDENTIFIER = sleevePipeline.getDestination();
        telemetry.addData("Parking", IDENTIFIER);
        telemetry.update();


        drive.followTrajectorySequence(trajectoryToJunction);
        //highJunctionRunPosition();
        highJunction();
        drive.followTrajectorySequence(trajectoryRecenter); //trajectoryRecenter ends in parking2


        if(IDENTIFIER == 1)
            drive.followTrajectorySequence(trajectoryToParking1);
        else if (IDENTIFIER == 3)
            drive.followTrajectorySequence(trajectoryToParking3);

    }


    /**
     * When the robot is in front of the High Junction, it
     * lifts the arm, approaches it, goes a bit down,
     * lets the cone go (opens claw), and lowers the lift back down.
     * Based on TIME
     */
    public void highJunction()
    {
        utilities.liftArm(1, 3166, telemetry); // .8 5300
        drive.followTrajectorySequence(goForward);
        utilities.lowerArm(1, 400, telemetry); //.8 500
        utilities.openClaw(true);
        utilities.lowerArm(1, 2666, telemetry); //.8 4800

    }


    /**
     * When the robot is in front of the High Junction, it
     * lifts the arm, approaches it, goes a bit down,
     * lets the cone go (opens claw), and lowers the lift back down.
     * Based on POSITION
     */
    public void highJunctionRunPosition()
    {
        hardware.liftArm.setTargetPosition(2100);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(goForward);
        hardware.liftArm.setTargetPosition(1900);
        utilities.openClaw(true);
        hardware.liftArm.setTargetPosition(500);
    }


    /**
     * Defines and builds the different trajectories used.
     */
    private void buildTrajectories()
    {
        trajectoryToJunction = drive.trajectorySequenceBuilder(HOME)
                .forward(26)
                .turn(Math.toRadians(36))
                .build();
        goForward = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                .forward(7.5)
                .build();
        trajectoryRecenter = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                .back(1.5)
                .turn(Math.toRadians(-127))
                .build();
        trajectoryToParking1 = drive.trajectorySequenceBuilder(trajectoryRecenter.end())
                .back(24)
                .build();
        trajectoryToParking3 = drive.trajectorySequenceBuilder(trajectoryRecenter.end())
                .forward(19)
                .build();

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
