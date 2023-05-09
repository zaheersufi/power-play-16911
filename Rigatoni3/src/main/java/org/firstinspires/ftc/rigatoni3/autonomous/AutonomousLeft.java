package org.firstinspires.ftc.rigatoni3.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.rigatoni3.drive.SampleMecanumDrive;
import org.firstinspires.ftc.rigatoni3.hardware.NewRigatoniHardware;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

@Autonomous(name = "AutonLeft")
public class AutonomousLeft extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private NewUtilities utilities;
    private NewRigatoniHardware hardware;

    private final Pose2d blueHome = new Pose2d(-36, -62.5, Math.toRadians(90));

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

    }

    public void buildTrajectories()
    {

    }
}

