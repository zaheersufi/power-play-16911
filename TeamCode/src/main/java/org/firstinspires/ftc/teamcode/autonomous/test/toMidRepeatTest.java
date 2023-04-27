package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.genericAuton;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name="toMidRepeatTest")
public class toMidRepeatTest extends genericAuton {

    private final Pose2d start = new Pose2d(63, 12, Math.toRadians(0));
    private final Pose2d atMid2Inbetween = new Pose2d(41, 12, Math.toRadians(110));
    private final Pose2d atMid2Final = new Pose2d(29, 19, Math.toRadians(135));
    private Trajectory toMidRepeat;
    @Override
    public void run() {
        drive.setPoseEstimate(start);
        drive.followTrajectory(toMidRepeat);
    }

    @Override
    public void buildTrajectories() {
        toMidRepeat = drive.trajectoryBuilder(start, Math.toRadians(180))
                .splineToSplineHeading(atMid2Inbetween, Math.toRadians(-180))
                .splineToSplineHeading(atMid2Final, Math.toRadians(100))
                .build();
    }
}
