package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.autonomous.Utilities;
import org.firstinspires.ftc.teamcode.autonomous.pipelines.JunctionPipeline;
import org.firstinspires.ftc.teamcode.autonomous.pipelines.SleevePipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.OldRigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;



@Disabled
@Autonomous(name="LeftTest")
public class LeftTest extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    private OldRigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline sleevePipeline;
    private JunctionPipeline junctionPipeline;

    private TrajectorySequence trajectoryToJunction;
    private TrajectorySequence goForward;
    private TrajectorySequence junctionCorrection;
    private TrajectorySequence trajectoryRecenter;
    private TrajectorySequence trajectoryToParking1;
    private TrajectorySequence trajectoryToParking3;


    private final Pose2d HOME = new Pose2d(36, 60, Math.toRadians(270));


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
        setUpCamera();
        webcam.setPipeline(new JunctionPipeline(telemetry));


        Assert.assertNotNull(hardwareMap);
        hardware = new OldRigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);
        utilities = new Utilities(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(HOME);


        turnOnEncoders();
        utilities.openClaw(false);
        waitForStart();
        if(!opModeIsActive()) return;
        utilities.wait(WAIT_TIME, telemetry);


        final int IDENTIFIER = sleevePipeline.getDestination();

        utilities.liftArm(1, 1400, telemetry);

        telemetry.addData("Parking", IDENTIFIER);
        telemetry.update();


        trajectoryToJunction = drive.trajectorySequenceBuilder(HOME)
                .forward(50)
                .turn(Math.toRadians(-49.5))
                .build();
        drive.followTrajectorySequence(trajectoryToJunction);


        junctionPipeline = new JunctionPipeline(telemetry);
        webcam.setPipeline(junctionPipeline);
        highJunction();
        drive.followTrajectorySequence(trajectoryRecenter); //trajectoryRecenter ends in parking2


        if(IDENTIFIER == 1)
            drive.followTrajectorySequence(trajectoryToParking1);
        else if (IDENTIFIER == 3)
            drive.followTrajectorySequence(trajectoryToParking3);

    }


    /**
     * When the robot is in front of the High Junction, it
     * lifts the arm, approaches it, reads the position of the
     * junction, corrects its position to align with the pole,
     * lets the cone go (opens claw), and lowers the lift back down.
     */
    public void highJunction ()
    {
        utilities.liftArm(1, 2000, telemetry);


        utilities.wait(100, telemetry);
        double displacement = junctionPipeline.getDisplacement(40);
        telemetry.addData("Displacement", displacement);
        telemetry.update();


        if (displacement > 1) {
            junctionCorrection = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                    .strafeRight(Math.abs(displacement)).build();
        } else if (displacement < -1) {
            junctionCorrection = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                    .strafeLeft(Math.abs(displacement)).build();
        } else {
            junctionCorrection = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                    .forward(0).build();
        }

        drive.followTrajectorySequence(junctionCorrection);
        buildTrajectories();


        utilities.liftArm(1, 1850, telemetry);
        drive.followTrajectorySequence(goForward);

        utilities.lowerArm(1, 400, telemetry);
        utilities.openClaw(true);
        utilities.lowerArm(1, 4250, telemetry);

    }


    /**
     * Defines and builds the different trajectories used.
     */
    private void buildTrajectories()
    {
        goForward = drive.trajectorySequenceBuilder(junctionCorrection.end())
                .forward(9.5)
                .build();
        trajectoryRecenter = drive.trajectorySequenceBuilder(trajectoryToJunction.end())
                .back(1.5)
                .turn(Math.toRadians(136))
                .build();
        trajectoryToParking1 = drive.trajectorySequenceBuilder(trajectoryRecenter.end())
                .forward(18)
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
     * Set up the webcam in an inverted horizontal position
     */
    public void setUpCamera()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
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
