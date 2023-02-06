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
@Autonomous(name="fullSendMidLeft")
public class fullSendMidLeft extends LinearOpMode
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


    private final Pose2d HOME = new Pose2d(36, 60, Math.toRadians(-90));



    /**
     * This method sets everything up for autonomous : it initializes components
     * and variables and calls the build trajectory method. After initialization,
     * the robot asynchronously lifts the arm up as it drives forward into the
     * junction through the use of  trajectoryToJunction. Then following trajectory
     * goForward, the cone is dropped onto the pylon. Then with the use of trajectory
     * reCenter, the robot parks based on the zone read from the signal sleeve during
     * the start time.
     *
     * Reads the parking position, scores a cone in the
     * mid junction, and parks in the space determined
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


        utilities.liftArmPosition(1520);

        drive.followTrajectorySequence(trajectoryToJunction);
        drive.followTrajectorySequence(goForward);

        utilities.liftArmPosition(-550);
        utilities.wait(500, telemetry);
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
     * This method is to be called in the runOpMode, and it sets up all trajectories
     * that will be used by the robot. It sets up trajectoryToJunction, goForward,
     * and trajectoryRecenter and sets up three other trajectories which are parking locations.
     *
     * Defines and builds the different trajectories used.
     */
    private void buildTrajectories()
    {
        trajectoryToJunction = drive.trajectorySequenceBuilder(HOME)
                .forward(26)
                .forward(3)
                .back(3)
                .turn(Math.toRadians(-43))
                .build();
        goForward = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                .forward(9)
                .build();
        trajectoryRecenter = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                .back(1.25)
                .turn(Math.toRadians(133))
                .build();
        trajectoryToParking1 = drive.trajectorySequenceBuilder(trajectoryRecenter.end())
                .forward(21)
                .build();
        trajectoryToParking3 = drive.trajectorySequenceBuilder(trajectoryRecenter.end())
                .back(26)
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
     * Sets up the camera/webcam and reading box which scans in the parking zone
     * through the use of color on our custom sleeve’s symbols.
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
