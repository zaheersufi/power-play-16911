package org.firstinspires.ftc.teamcode.autonomous.spline;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.autonomous.NewUtilities;
import org.firstinspires.ftc.teamcode.autonomous.Utilities;
import org.firstinspires.ftc.teamcode.autonomous.pipelines.SleevePipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;
import org.firstinspires.ftc.teamcode.hardware.OldRigatoniHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


//@Disabled
@Autonomous(name="leftMultiple")
public class leftMultiple extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private NewUtilities utilities;
    private NewRigatoniHardware hardware;
    private OpenCvInternalCamera webcam;
    private SleevePipeline sleevePipeline;


    private final Pose2d home = new Pose2d(-36.0, -63.0, Math.toRadians(90.0));

    private final Pose2d midJunction = new Pose2d(-34.0, -24.0, Math.toRadians(0.0));

    private final Pose2d stack = new Pose2d(-59.0, -12.0, Math.toRadians(180.0));

    private final Pose2d lowJunction = new  Pose2d(-55.0, -16.0, Math.toRadians(-50.0));
    private final Pose2d groundJunction = new Pose2d(-55.5, -8.5, Math.toRadians(50.0));
    private final Pose2d highJunction = new Pose2d(-31.5, -7.5, Math.toRadians(45.0));

    private final Pose2d parkingOne = new Pose2d(-59.0, -13.0, Math.toRadians(180.0));
    private final Pose2d parkingTwo =  new Pose2d(-36.0, -12.0, Math.toRadians(180.0));
    private final Pose2d parkingThree = new Pose2d(-13.25, -12.0, Math.toRadians(180.0));


    private Trajectory trajectoryToMid;
    private Trajectory trajectoryMidForward;
    private Trajectory trajectoryMidBackward;
    private Trajectory trajectoryMidToStack;
    private Trajectory trajectoryPickUpCone;
    private Trajectory trajectoryAfterPickUp;
    private Trajectory trajectoryToLow;
    private Trajectory trajectoryLowForward;
    private Trajectory trajectoryLowBackward;
    private Trajectory trajectoryLowToStack;
    private Trajectory trajectoryToGround;
    private Trajectory trajectoryGroundForward;
    private Trajectory trajectoryGroundBackward;
    private Trajectory trajectoryGroundToStack;
    private Trajectory trajectoryToHigh;
    private Trajectory trajectoryHighForward;
    private Trajectory trajectoryHighBackward;
    private Trajectory trajectoryToParkingOne;
    private Trajectory trajectoryToParkingTwo;
    private Trajectory trajectoryToParkingThree;



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
        hardware = new NewRigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);
        hardware.turnOnDriveEncoders();
        utilities = new NewUtilities(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(home);


        buildTrajectories();
        utilities.openClaw(false);
        waitForStart();
        if(!opModeIsActive()) {return;}
        utilities.wait(250, telemetry);


        final int IDENTIFIER = sleevePipeline.getDestination();
        telemetry.addData("Parking", IDENTIFIER);
        telemetry.update();


        utilities.liftArmPosition(1480);
        drive.followTrajectory(trajectoryToMid);
        drive.followTrajectory(trajectoryMidForward);
        utilities.liftArmPosition(-500);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectory(trajectoryMidBackward);

        drive.followTrajectory(trajectoryMidToStack);
        drive.followTrajectory(trajectoryPickUpCone);
        utilities.liftArmPosition(-200);
        utilities.wait(500, telemetry);
        utilities.openClaw(false);
        drive.followTrajectory(trajectoryAfterPickUp);


        utilities.liftArmPosition(800);
        drive.followTrajectory(trajectoryToLow);
        drive.followTrajectory(trajectoryLowForward);
        utilities.liftArmPosition(-500);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectory(trajectoryLowBackward);

        drive.followTrajectory(trajectoryLowToStack);
        drive.followTrajectory(trajectoryPickUpCone);
        utilities.liftArmPosition(-200);
        utilities.wait(500, telemetry);
        utilities.openClaw(false);
        drive.followTrajectory(trajectoryAfterPickUp);


        utilities.liftArmPosition(-200);
        drive.followTrajectory(trajectoryToGround);
        drive.followTrajectory(trajectoryGroundForward);
        utilities.liftArmPosition(-300);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectory(trajectoryGroundBackward);

        drive.followTrajectory(trajectoryGroundToStack);
        drive.followTrajectory(trajectoryPickUpCone);
        utilities.liftArmPosition(-200);
        utilities.wait(500, telemetry);
        utilities.openClaw(false);
        drive.followTrajectory(trajectoryAfterPickUp);


        utilities.liftArmPosition(1600);
        drive.followTrajectory(trajectoryToHigh);
        drive.followTrajectory(trajectoryHighForward);
        utilities.liftArmPosition(-500);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectory(trajectoryHighBackward);
        utilities.liftArmPosition(-1600);


        if(IDENTIFIER == 1)
            drive.followTrajectory(trajectoryToParkingOne);
        else if (IDENTIFIER == 2)
            drive.followTrajectory(trajectoryToParkingTwo);
        else if (IDENTIFIER == 3)
            drive.followTrajectory(trajectoryToParkingThree);

    }


    /**
     * Defines and builds the different trajectories used.
     */
    private void buildTrajectories()
    {

        trajectoryToMid = drive.trajectoryBuilder(home, Math.toRadians(80.0))
                .splineToSplineHeading(midJunction, Math.toRadians(75.0))
                .build();
        trajectoryMidForward = drive.trajectoryBuilder(trajectoryToMid.end(), trajectoryToMid.end().getHeading())
                .forward(2.5)
                .build();
        trajectoryMidBackward = drive.trajectoryBuilder(trajectoryMidForward.end(), trajectoryMidForward.end().getHeading())
                .back(3.0)
                .build();
        trajectoryMidToStack = drive.trajectoryBuilder(trajectoryMidBackward.end(), Math.toRadians(80.0))
                .splineToSplineHeading(stack, Math.toRadians(173.0))
                .build();


        trajectoryPickUpCone = drive.trajectoryBuilder(stack, Math.toRadians(180.0))
                .forward(4.0)
                .build();
        trajectoryAfterPickUp = drive.trajectoryBuilder(trajectoryPickUpCone.end(), Math.toRadians(0.0))
                .splineToSplineHeading(stack, Math.toRadians(0.0))
                .build();


        trajectoryToLow = drive.trajectoryBuilder(stack, Math.toRadians(-45.0))
                .splineToSplineHeading(lowJunction, lowJunction.getHeading())
                .build();
        trajectoryLowForward = drive.trajectoryBuilder(trajectoryToLow.end(), trajectoryToLow.end().getHeading())
                .forward(4.5)
                .build();
        trajectoryLowBackward = drive.trajectoryBuilder(trajectoryLowForward.end(), trajectoryLowForward.end().getHeading())
                .back(4.5)
                .build();
        trajectoryLowToStack = drive.trajectoryBuilder(trajectoryLowBackward.end(), Math.toRadians(210.0))
                .splineToSplineHeading(stack, stack.getHeading())
                .build();


        trajectoryToGround = drive.trajectoryBuilder(stack, Math.toRadians(45.0))
                .splineToSplineHeading(groundJunction, groundJunction.getHeading())
                .build();
        trajectoryGroundForward = drive.trajectoryBuilder(trajectoryToGround.end(), trajectoryToGround.end().getHeading())
                .forward(5.0)
                .build();
        trajectoryGroundBackward = drive.trajectoryBuilder(trajectoryGroundForward.end(), trajectoryGroundForward.end().getHeading())
                .back(5.0)
                .build();
        trajectoryGroundToStack = drive.trajectoryBuilder(trajectoryGroundBackward.end(), Math.toRadians(-225.0))
                .splineToSplineHeading(stack, stack.getHeading())
                .build();


        trajectoryToHigh = drive.trajectoryBuilder(stack, Math.toRadians(0.0))
                .splineToSplineHeading(highJunction, Math.toRadians(60.0))
                .build();
        trajectoryHighForward = drive.trajectoryBuilder(trajectoryToHigh.end(), trajectoryToHigh.end().getHeading())
                .forward(3.0)
                .build();
        trajectoryHighBackward = drive.trajectoryBuilder(trajectoryHighForward.end(), trajectoryHighForward.end().getHeading())
                .back(2.5)
                .build();


        trajectoryToParkingOne = drive.trajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-150.0))
                .splineToSplineHeading(parkingOne, Math.toRadians(175.0))
                .build();
        trajectoryToParkingTwo = drive.trajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-120.0))
                .splineToSplineHeading(parkingTwo, Math.toRadians(135.0))
                .build();
        trajectoryToParkingThree = drive.trajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-45.0))
                .splineToSplineHeading(parkingThree, Math.toRadians(0.0))
                .build();
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
