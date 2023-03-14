package org.firstinspires.ftc.teamcode.autonomous.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autonomous.genericAuton;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="newLeft")
public class newLeft extends genericAuton
{
    //toMid1
    private final Pose2d blueLeftHome = new Pose2d(36.0, 62.5, Math.toRadians(-90.0));
    private final Pose2d toMid1bet1 = new Pose2d(36, 59, Math.toRadians(-90));
    private final Pose2d toMid1bet2 = new Pose2d(36, 50, Math.toRadians(180));
    private final Pose2d toMid1bet3 = new Pose2d(36, 23, Math.toRadians(180));

    //toMidFinal
    private final Pose2d toMidFinalbet1 = new Pose2d(65, 12, Math.toRadians(0));
    private final Pose2d toMidFinalbet2 = new Pose2d(42, 12, Math.toRadians(0));
    private final Pose2d toMidFinalbet3 = new Pose2d(29, 12, Math.toRadians(90));
//  private final Pose2d toMidFinalbet4 = new Pose2d(29, 12, Math.toRadians(90));

    private Trajectory toMid1;
    private TrajectorySequence forwardMid1;
    private TrajectorySequence backMid1;
    private TrajectorySequence toStack1;
    private TrajectorySequence toLow1;
    private TrajectorySequence lowBack1;
    private TrajectorySequence toLow2;
    private TrajectorySequence lowBack2;
    private Trajectory toFinalMid;
    private TrajectorySequence parking1;
    private TrajectorySequence parking2;
    private TrajectorySequence parking3;


    @Override
    public void run()
    {
        drive.setPoseEstimate(blueLeftHome);

        //go to mid1
        utilities.liftArmAbsolutePosition(270);
        utilities.wait(100, telemetry);
        utilities.tiltClaw(false);
        drive.followTrajectory(toMid1);
        //drive.followTrajectorySequence(forwardMid1);
        utilities.openClaw(true);
        //drive.followTrajectorySequence(backMid1);

        //toStack1
        utilities.liftArmAbsolutePosition(75);
        utilities.tiltClaw(true);
        drive.followTrajectorySequence(toStack1);
        utilities.openClaw(false);
        utilities.wait(500, telemetry);
        utilities.liftArmAbsolutePosition(185);
        utilities.wait(500, telemetry);
        utilities.tiltClaw(false);

        //toLow1
        drive.followTrajectorySequence(toLow1);
        utilities.openClaw(true);
        utilities.tiltClaw(true);
        utilities.liftArmAbsolutePosition(60);
        drive.followTrajectorySequence(lowBack1);

        //toLow2
        utilities.openClaw(false);
        utilities.wait(500, telemetry);
        utilities.liftArmAbsolutePosition(185);
        utilities.wait(500, telemetry);
        utilities.tiltClaw(false);
        drive.followTrajectorySequence(toLow2);
        utilities.openClaw(true);
        utilities.tiltClaw(true);
        utilities.liftArmAbsolutePosition(50);
        drive.followTrajectorySequence(lowBack2);

        //toFinalMid
        utilities.openClaw(false);
        utilities.wait(500, telemetry);
        utilities.liftArmAbsolutePosition(240);
        utilities.wait(500, telemetry);
        utilities.tiltClaw(false);
        drive.followTrajectory(toFinalMid);
        drive.followTrajectorySequence(forwardMid1);
        utilities.openClaw(true);

        //parking
        if(identifier == 1)
            drive.followTrajectorySequence(parking1);
        else if (identifier == 2)
            drive.followTrajectorySequence(parking2);
        else if (identifier == 3)
            drive.followTrajectorySequence(parking3);


    }

    @Override
    public void buildTrajectories()
    {
        toMid1 = drive.trajectoryBuilder(blueLeftHome, Math.toRadians(-90))
                .splineToSplineHeading(toMid1bet1, Math.toRadians(-90))
                .splineToSplineHeading(toMid1bet2, Math.toRadians(-90))
                .splineToSplineHeading(toMid1bet3, Math.toRadians(-90))
                .build();
//        forwardMid1 = drive.trajectorySequenceBuilder(toFinalMid.end())
//                .forward(3)
//                .build();
//        backMid1 = drive.trajectorySequenceBuilder(forwardMid1.end())
//                .back(3)
//                .build();
        toStack1 = drive.trajectorySequenceBuilder(toMid1.end())
                .strafeLeft(11)
                .turn(Math.toRadians(180))
                .forward(31.5)
                .build();
        toLow1 = drive.trajectorySequenceBuilder(toStack1.end())
                .back(8)
                .turn(Math.toRadians(115))
                .forward(7.5)
                .build();
        lowBack1 = drive.trajectorySequenceBuilder(toLow1.end())
                .back(7.5)
                .turn(Math.toRadians(-115))
                .forward(8)
                .build();
        toLow2 = drive.trajectorySequenceBuilder(lowBack1.end())
                .back(8)
                .turn(Math.toRadians(115))
                .forward(7.5)
                .build();
        lowBack2 = drive.trajectorySequenceBuilder(toLow2.end())
                .back(7.5)
                .turn(Math.toRadians(-115))
                .forward(8)
                .build();
        toFinalMid = drive.trajectoryBuilder(toMidFinalbet1, Math.toRadians(180))
                .splineToSplineHeading(toMidFinalbet2, Math.toRadians(180))
                .splineToSplineHeading(toMidFinalbet3, Math.toRadians(180))
//                .splineToSplineHeading(toMidFinalbet4, Math.toRadians(180))
                .build();
        forwardMid1 = drive.trajectorySequenceBuilder(toFinalMid.end())
                .forward(4)
                .build();
        parking1 = drive.trajectorySequenceBuilder(toFinalMid.end())
                .strafeRight(38)
                .build();
        parking2 = drive.trajectorySequenceBuilder(toFinalMid.end())
                .strafeRight(12)
                .build();
        parking3 = drive.trajectorySequenceBuilder(toFinalMid.end())
                .strafeLeft(12)
                .build();

    }
}
