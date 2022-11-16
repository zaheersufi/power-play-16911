package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Test Autonomous")
public class TestAuton extends LinearOpMode {
    private SampleMecanumDrive drive;
    private RigatoniHardware hardware;
    private TrajectorySequence trajectoryToParking1;
    private Utilities utilities;
    private final Pose2d blueHome = new Pose2d(-36, 60, Math.toRadians(270));
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
        drive.followTrajectorySequence(trajectoryToParking1);
    }
    private void turnOnEncoders(RigatoniHardware hardware)
    {
        hardware.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.liftArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    private void buildTrajectories() {
        trajectoryToParking1 = drive.trajectorySequenceBuilder(blueHome)
                .forward(30)
                .build();

    }
}
