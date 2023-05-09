package org.firstinspires.ftc.teamcode.autonomous.spline;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.Utilities;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.OldRigatoniHardware;

@Autonomous(name = "UnderclassmenAuton")
public class UnderclassmanAuton extends LinearOpMode
{
    OldRigatoniHardware hardware = new OldRigatoniHardware();
    //Utilities utilities = new Utilities(hardware);
    SampleMecanumDrive drive;

    Pose2d pose1 = new Pose2d(-36, 60, Math.toRadians(-90));
    Pose2d pose2 = new Pose2d(-38, 24, Math.toRadians(-90));
    Pose2d pose3 = new Pose2d(-48, 12, Math.toRadians(180));
    Pose2d pose4 = new Pose2d(-60, 12, Math.toRadians(180));

    Trajectory trajectory1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new SampleMecanumDrive(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializePrimaryMotors(hardwareMap);
        //hardware.initializeSupplementaryMotors(hardwareMap);

        drive.setPoseEstimate(pose1);
        buildTrajectories();
        waitForStart();

        drive.followTrajectory(trajectory1);
    }

    public void buildTrajectories ()
    {
        trajectory1 = drive.trajectoryBuilder(pose1, Math.toRadians(-90))
                .splineToSplineHeading(pose2, Math.toRadians(-90))
                   .splineToSplineHeading(pose3, Math.toRadians(180))
                .splineToSplineHeading(pose4, Math.toRadians(180))
                .build();
    }
}
