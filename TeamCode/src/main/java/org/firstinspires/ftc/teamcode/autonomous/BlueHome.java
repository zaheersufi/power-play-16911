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

@Autonomous(name="BlueHome")
public class BlueHome extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;
    private RobotVision robotVision;
    RigatoniHardware hardware;

    private int initialWaitTime = 0;

    private final Pose2d blueHome = new Pose2d(-36, 60, Math.toRadians(270));

    private TrajectorySequence trajectoryTo12; //check coordinate system in notebook
    private TrajectorySequence trajectoryToParking3;
    private TrajectorySequence trajectoryToParking2;
    private TrajectorySequence trajectoryToParking1;

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

        drive.followTrajectorySequence(trajectoryTo12);
        // utilities.wait(1200, telemetry);
        utilities.highJunction(telemetry);
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
    private void buildTrajectories()
    {
        trajectoryTo12 = drive.trajectorySequenceBuilder(blueHome)
                .turn(Math.toRadians(90))
                .forward(24)
                .strafeRight(34)
                .build();
        trajectoryToParking3 = drive.trajectorySequenceBuilder(blueHome)
                .strafeRight(14)
                .turn(Math.toRadians(180))
                .forward(47)
                .build();
        trajectoryToParking2 = drive.trajectorySequenceBuilder(blueHome)
                .strafeRight(14)
                .turn(Math.toRadians(180))
                .forward(24)
                .build();
        trajectoryToParking1 = drive.trajectorySequenceBuilder(blueHome)
                .strafeRight(14)
                .turn(Math.toRadians(180))
                .build();
    }
}
