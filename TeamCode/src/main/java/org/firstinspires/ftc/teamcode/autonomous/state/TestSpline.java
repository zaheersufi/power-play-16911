package org.firstinspires.ftc.teamcode.autonomous.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.genericAuton;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name = "testSpline")
public class TestSpline extends genericAuton {

    private Trajectory testSpline;
    private final Pose2d home = new Pose2d(36.0, 62.5, Math.toRadians(270));
    private final Pose2d secondPoint = new Pose2d(36, 25, Math.toRadians(180));
    @Override
    public void run() {
        drive.setPoseEstimate(home);
        drive.followTrajectory(testSpline);

    }

    @Override
    public void buildTrajectories() {
        testSpline = drive.trajectoryBuilder(home, Math.toRadians(-90))
                .splineToSplineHeading(secondPoint, Math.toRadians(270))
                .build();
    }
}
