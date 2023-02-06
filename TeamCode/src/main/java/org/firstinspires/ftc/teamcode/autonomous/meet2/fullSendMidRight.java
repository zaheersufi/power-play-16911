package org.firstinspires.ftc.teamcode.autonomous.meet2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.autonomous.pipelines.SleevePipeline;
import org.firstinspires.ftc.teamcode.autonomous.Utilities;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.OldRigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
@Autonomous(name="fullSendMidRight")
public class fullSendMidRight extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    private OldRigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline sleevePipeline;


    private TrajectorySequence trajectoryToJunction;
    private TrajectorySequence goForward;
    private TrajectorySequence trajectoryRecenter;
    private TrajectorySequence trajectoryToParking1;
    private TrajectorySequence trajectoryToParking3;


    private final Pose2d HOME = new Pose2d(-36, 60, Math.toRadians(270));



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
        hardware = new OldRigatoniHardware();
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
        utilities.wait(250, telemetry);


        final int IDENTIFIER = sleevePipeline.getDestination();
        telemetry.addData("Parking", IDENTIFIER);
        telemetry.update();


        utilities.liftArmPosition(1500);

        drive.followTrajectorySequence(trajectoryToJunction);
        drive.followTrajectorySequence(goForward);

        utilities.liftArmPosition(-580);
        utilities.wait(750, telemetry);
        utilities.openClaw(true);
        utilities.liftArmPosition(-1500);

        //trajectoryRecenter ends in parking2
        drive.followTrajectorySequence(trajectoryRecenter);


        if(IDENTIFIER == 1)
            drive.followTrajectorySequence(trajectoryToParking1);
        else if (IDENTIFIER == 3)
            drive.followTrajectorySequence(trajectoryToParking3);

    }


    /**
     * Defines and builds the different trajectories used.
     */
    private void buildTrajectories()
    {
        trajectoryToJunction = drive.trajectorySequenceBuilder(HOME)
                .forward(26)
                .forward(3)
                .back(3.5)
                .turn(Math.toRadians(36))
                .build();
        goForward = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                .forward(7.5)
                .build();
        trajectoryRecenter = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                .back(0.0625)
                .turn(Math.toRadians(-127))
                .build();
        trajectoryToParking1 = drive.trajectorySequenceBuilder(trajectoryRecenter.end())
                .back(24)
                .build();
        trajectoryToParking3 = drive.trajectorySequenceBuilder(trajectoryRecenter.end())
                .forward(23)
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
