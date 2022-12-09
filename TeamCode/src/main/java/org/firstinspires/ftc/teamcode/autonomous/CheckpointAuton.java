package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Autonomous(name="Checkpoint Autonomous")
public class CheckpointAuton extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    RigatoniHardware hardware;

    private int initialWaitTime = 0;

    private final Pose2d blueHome = new Pose2d(-36, 60, Math.toRadians(270));

    private TrajectorySequence trajectoryTo12; //check coordinate system in notebook
    private TrajectorySequence trajectoryToParking3;
    private TrajectorySequence trajectoryToParking2;
    private TrajectorySequence trajectoryToParking1;
    private TrajectorySequence trajectorySlamCone;
    private TrajectorySequence trajectoryMoveForward;

    @Override
    public void runOpMode()
    {
        hardware = new RigatoniHardware(); //Horizontal Claw
        Assert.assertNotNull(hardwareMap);
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);

        turnOnEncoders(hardware);
        //robotVision = new RobotVision(hardwareMap);
        //int identifier = robotVision.identify();
        int identifier = 0;
        utilities = new Utilities(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(blueHome);
        buildTrajectories();

        waitForStart();
        if(!opModeIsActive()) {return;}

        utilities.openClaw(false);
        utilities.wait(initialWaitTime, telemetry);

        drive.followTrajectorySequence(trajectorySlamCone);

    }
    private void turnOnEncoders(RigatoniHardware hardware)
    {
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void buildTrajectories()
    {
        trajectorySlamCone = drive.trajectorySequenceBuilder(blueHome)
                .strafeRight(16)
                .forward(3)
                .build();
    }
}
