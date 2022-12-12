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

@Autonomous(name="NewLeft")
public class NewLeft extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    private RigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline sleevePipeline;
    private JunctionPipeline junctionPipeline;

    private TrajectorySequence trajectoryTo12; //check coordinate system in notebook
    private TrajectorySequence trajectoryToParking3;
    private TrajectorySequence trajectoryToParking2;
    private TrajectorySequence trajectoryToParking1;
    private TrajectorySequence goForward;


    private final int WAIT_TIME = 250;
    private final Pose2d BLUEHOME = new Pose2d(-36, -60, Math.toRadians(90));



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
        webcam.setPipeline(sleevePipeline);


        Assert.assertNotNull(hardwareMap);
        hardware = new RigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);
        turnOnEncoders();

        utilities = new Utilities(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(BLUEHOME);


        buildTrajectories();
        utilities.openClaw(false);


        waitForStart();
        if(!opModeIsActive()) return;
        utilities.wait(WAIT_TIME, telemetry);


        int identifier = sleevePipeline.getDestination();
        telemetry.addData("Parking", identifier);
        telemetry.update();

        drive.followTrajectorySequence(trajectoryTo12);


        junctionPipeline = new JunctionPipeline(telemetry);
        webcam.setPipeline(junctionPipeline);
        highJunction();


        if(identifier == 1)
            drive.followTrajectorySequence(trajectoryToParking1);
        else if (identifier == 2)
            drive.followTrajectorySequence(trajectoryToParking2);
        else
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
        utilities.liftArm(.8, 5300, telemetry);
        drive.followTrajectorySequence(goForward);


        utilities.wait(100, telemetry);
        double displacement = junctionPipeline.getDisplacement(10);

        if (displacement > 1) {
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(goForward.end())
                    .strafeRight(Math.abs(displacement)).build());
        } else if (displacement < -1) {
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(goForward.end())
                    .strafeLeft(Math.abs(displacement)).build());
        }


        utilities.lowerArm(.8, 500, telemetry);
        utilities.openClaw(true);
        utilities.lowerArm(.8, 4800, telemetry);

    }



    /**
     * Defines and builds the different trajectories used.
     */
    private void buildTrajectories()
    {
        trajectoryTo12 = drive.trajectorySequenceBuilder(BLUEHOME)
                .forward(6)
                .turn(Math.toRadians(-90))
                .forward(21.25)
                .strafeLeft(38)
                .forward(0.5)
                .build();

        goForward = drive.trajectorySequenceBuilder(trajectoryTo12.end()) //trajectoryTo12.end()
                .forward(5)
                .build();

        trajectoryToParking1 = drive.trajectorySequenceBuilder(goForward.end()) //goForward.end()
                .strafeLeft(12)
                .turn(Math.toRadians(180))
                .forward(43.5)
                .strafeLeft(1)
                .build();

        trajectoryToParking2 = drive.trajectorySequenceBuilder(goForward.end()) //goForward.end()
                .strafeLeft(12)
                .turn(Math.toRadians(180))
                .forward(22)
                .build();

        trajectoryToParking3 = drive.trajectorySequenceBuilder(goForward.end()) //beforeJunction goForward.end())
                .strafeLeft(12)
                .turn(Math.toRadians(180))
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
