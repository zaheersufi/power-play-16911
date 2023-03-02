package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.genericAuton;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RotationPositionTest extends genericAuton
{
    private final Pose2d home = new Pose2d(-36.0, -63.0, Math.toRadians(90.0));
    private TrajectorySequence turn30;
    private TrajectorySequence turn60;

    @Override
    public void run() {
        drive.setPoseEstimate(home);
        telemetry.addData("Rotational Orientation", drive.getImu().getAngularOrientation().toString());
        drive.followTrajectorySequence(turn30);
        telemetry.addData("Rotational Orientation", drive.getImu().getAngularOrientation().toString());
        drive.followTrajectorySequence(turn60);
        telemetry.addData("Rotational Orientation", drive.getImu().getAngularOrientation().toString());
    }

    @Override
    public void buildTrajectories() {
        turn30 = drive.trajectorySequenceBuilder(home)
                .turn(Math.toRadians(30))
                .build();
        turn60 = drive.trajectorySequenceBuilder(home)
                .turn(Math.toRadians(60))
                .build();

    }
}