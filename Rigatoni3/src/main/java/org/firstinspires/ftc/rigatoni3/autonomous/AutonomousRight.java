package org.firstinspires.ftc.rigatoni3.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.rigatoni3.drive.SampleMecanumDrive;
import org.firstinspires.ftc.rigatoni3.hardware.NewRigatoniHardware;
import org.firstinspires.ftc.rigatoni3.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

@Autonomous(name = "AutonRight")
public class AutonomousRight extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private NewUtilities utilities;
    private NewRigatoniHardware hardware;

    private final Pose2d blueHome = new Pose2d(-36, 62.5, Math.toRadians(-180));//heading
    private final Pose2d toMid1 = new Pose2d(-36, 15, Math.toRadians(-140));
    private final Pose2d toStack = new Pose2d(-60, 12, Math.toRadians(180));
    private final Pose2d toMid2 = new Pose2d(-36, 15, Math.toRadians(-130));

    private Trajectory toMid;
    private Trajectory toStackRepeat;
    private Trajectory toMidRepeat;
    private TrajectorySequence parking;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Assert.assertNotNull(hardwareMap);

        hardware = new NewRigatoniHardware();
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);
        utilities = new NewUtilities(hardware);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(blueHome);

        utilities.openClaw(false);
        buildTrajectories();

        run();

    }
    public void run()
    {
        drive.setPoseEstimate(blueHome);

        //toMid
        //utilities.liftArmAbsolutePosition(240);
        //utilities.shouldFlip(false);
        utilities.wait(1000, telemetry);
        drive.followTrajectory(toMid);
        //utilities.openClaw(true);
        utilities.wait(200, telemetry);

        //toStack1
        //utilities.liftArmAbsolutePosition(80);
        //utilities.shouldFlip(true);
        drive.followTrajectory(toStackRepeat);
        //utilities.openClaw(false);
        utilities.wait(200, telemetry);

        //toMidRepeat1
        //utilities.liftArmAbsolutePosition(240);
        //utilities.shouldFlip(false);
        utilities.wait(1000, telemetry);
        drive.followTrajectory(toMidRepeat);
        //utilities.openClaw(true);
        utilities.wait(200, telemetry);

        //toStack2
        //utilities.liftArmAbsolutePosition(70);
        //utilities.shouldFlip(true);
        drive.followTrajectory(toStackRepeat);
        //utilities.openClaw(false);
        utilities.wait(200, telemetry);

        //toMidRepeat2
        //utilities.liftArmAbsolutePosition(240);
        //utilities.shouldFlip(false);
        utilities.wait(1000, telemetry);
        drive.followTrajectory(toMidRepeat);
        //utilities.openClaw(true);
        utilities.wait(200, telemetry);

        //toStack3
        //utilities.liftArmAbsolutePosition(60);
        //utilities.shouldFlip(true);
        drive.followTrajectory(toStackRepeat);
        //utilities.openClaw(false);
        utilities.wait(200, telemetry);

        //toMidRepeat3
        //utilities.liftArmAbsolutePosition(240);
        //utilities.shouldFlip(false);
        utilities.wait(1000, telemetry);
        drive.followTrajectory(toMidRepeat);
        //utilities.openClaw(true);
        utilities.wait(200, telemetry);

        //toStack4
        //utilities.liftArmAbsolutePosition(50);
        //utilities.shouldFlip(true);
        drive.followTrajectory(toStackRepeat);
        //utilities.openClaw(false);
        utilities.wait(200, telemetry);

        //toMidRepeat4
        //utilities.liftArmAbsolutePosition(240);
        //utilities.shouldFlip(false);
        utilities.wait(1000, telemetry);
        drive.followTrajectory(toMidRepeat);
        //utilities.openClaw(true);
        utilities.wait(200, telemetry);

        //toStack5
        //utilities.liftArmAbsolutePosition(40);
        //utilities.shouldFlip(true);
        drive.followTrajectory(toStackRepeat);
        //utilities.openClaw(false);
        utilities.wait(200, telemetry);

        //toMidRepeat5
        //utilities.liftArmAbsolutePosition(240);
        //utilities.shouldFlip(false);
        utilities.wait(1000, telemetry);
        drive.followTrajectory(toMidRepeat);
        //utilities.openClaw(true);
        utilities.wait(200, telemetry);

        //parking
        drive.followTrajectorySequence(parking);
    }

    public void buildTrajectories()
    {
        toMid = drive.trajectoryBuilder(blueHome, Math.toRadians(-90))
                .splineToSplineHeading(toMid1, Math.toRadians(-90))
                .build();

        toStackRepeat = drive.trajectoryBuilder(toMid.end(), Math.toRadians(-160))
                .splineToSplineHeading(toStack, Math.toRadians(180))
                .build();

        toMidRepeat = drive.trajectoryBuilder(toStackRepeat.end(), Math.toRadians(0))
                .splineToSplineHeading(toMid2, Math.toRadians(20))
                .build();

        parking = drive.trajectorySequenceBuilder(toMidRepeat.end())
                .turn(Math.toRadians(-40))
                .build();
    }
}
