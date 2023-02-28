package org.firstinspires.ftc.teamcode.autonomous.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.genericAuton;

public class blueLeftFour extends genericAuton
{
    //toMid1
    private final Pose2d blueLeftHome = new Pose2d(36.0, 62.5, Math.toRadians(-90.0));
    private final Pose2d atMid1 = new Pose2d(29, 29, Math.toRadians(225));

    //toStack1
    private final Pose2d atStack1Inbetween = new Pose2d(39, 18, Math.toRadians(75));
    private final Pose2d atStack1Final = new Pose2d(63, 12, Math.toRadians(0));

    //toMidRepeat
    private final Pose2d atMid2Inbetween = new Pose2d(41, 12, Math.toRadians(110));
    private final Pose2d atMid2Final = new Pose2d(29, 19, Math.toRadians(135));

    //toStackRepeat
    private final Pose2d atStack2Inbetween1 = new Pose2d(31, 18, Math.toRadians(135));
    private final Pose2d atStack2Inbetween2 = new Pose2d(38, 14, Math.toRadians(20));
    private final Pose2d atStack2Inbetween3 = new Pose2d(63, 12, Math.toRadians(0));

    private Trajectory toMid1;
    private Trajectory toStack1;
    private Trajectory toMidRepeat;
    private Trajectory toStackRepeat;

    @Override
    public void run()
    {
        drive.setPoseEstimate(blueLeftHome);
    }

    @Override
    public void buildTrajectories()
    {
        toMid1 = drive.trajectoryBuilder(blueLeftHome, Math.toRadians(-90))
                .splineToLinearHeading(atMid1, Math.toRadians(225))
                .build();
        toStack1 = drive.trajectoryBuilder(toMid1.end())
                .splineToSplineHeading(atStack1Inbetween, Math.toRadians(-45))
                .splineToSplineHeading(atStack1Final, Math.toRadians(-6))
                .build();
    }
}
