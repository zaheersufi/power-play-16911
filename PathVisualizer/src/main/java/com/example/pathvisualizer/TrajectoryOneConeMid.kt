package com.example.pathvisualizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.example.pathvisualizer.TrajectoryTemplate


class TrajectoryOneConeMid : TrajectoryTemplate() {

    override fun left(dest: Int): ArrayList<Trajectory> {
        val trajectories = ArrayList<Trajectory>()


        val leftHome = Pose2d(36.0, 62.5, Math.toRadians(-90.0))

        val otwJunction = Pose2d(12.0, 48.0, Math.toRadians(-90.0))
        val inFrontOfJunction = Pose2d(16.0, 24.0, Math.toRadians(0.0))

        val otw1One = Pose2d(20.0, 13.0, Math.toRadians(0.0))
        val otw2One = Pose2d(36.0, 12.0, Math.toRadians(0.0))
        val parkingOne = Pose2d(58.5, 12.0, Math.toRadians(0.0))

        val otwTwo = Pose2d(20.0, 12.0, Math.toRadians(0.0))
        val parkingTwo = Pose2d(36.0, 12.0, Math.toRadians(0.0))

        val parkingThree = Pose2d(13.25, 12.0, Math.toRadians(0.0))


        var trajectoryToJunction = TrajectoryBuilder(leftHome, Math.toRadians(-180.0), combinedConstraints)
            .splineToSplineHeading(otwJunction, Math.toRadians(-80.0))
            .splineToSplineHeading(inFrontOfJunction, Math.toRadians(-60.0))
            .build();

        var trajectoryGoForward = TrajectoryBuilder(trajectoryToJunction.end(), trajectoryToJunction.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        var trajectoryGoBackward = TrajectoryBuilder(trajectoryGoForward.end(), trajectoryGoForward.end().heading, combinedConstraints)
            .back(6.0)
            .build();

        var trajectoryToParkingOne = TrajectoryBuilder(trajectoryGoBackward.end(), Math.toRadians(-90.0), combinedConstraints)
            .splineToSplineHeading(otw1One, Math.toRadians(-10.0))
            .splineToSplineHeading(otw2One, Math.toRadians(0.0))
            .splineToSplineHeading(parkingOne, Math.toRadians(0.0))
            .build();

        var trajectoryToParkingTwo = TrajectoryBuilder(trajectoryGoBackward.end(), Math.toRadians(-90.0), combinedConstraints)
            .splineToSplineHeading(otwTwo, Math.toRadians(-10.0))
            .splineToSplineHeading(parkingTwo, Math.toRadians(0.0))
            .build();

        var trajectoryToParkingThree = TrajectoryBuilder(trajectoryGoBackward.end(), Math.toRadians(-90.0), combinedConstraints)
            .splineToSplineHeading(parkingThree, Math.toRadians(-90.0))
            .build();


        trajectories.add(trajectoryToJunction)
        trajectories.add(trajectoryGoForward)
        trajectories.add(trajectoryGoBackward)

        when (dest) {
            1 -> trajectories.add(trajectoryToParkingOne)
            2 -> trajectories.add(trajectoryToParkingTwo)
            3 -> trajectories.add(trajectoryToParkingThree)
        }


        return trajectories
    }



    override fun right(dest: Int): ArrayList<Trajectory> {
        val trajectories = ArrayList<Trajectory>()


        val rightHome = Pose2d(36.0, -62.5, Math.toRadians(90.0))

        val otwJunction = Pose2d(12.0, -48.0, Math.toRadians(90.0))
        val inFrontOfJunction = Pose2d(16.0, -24.0, Math.toRadians(0.0))

        val parkingOne = Pose2d(13.25, -12.0, Math.toRadians(0.0))

        val otwTwo = Pose2d(20.0, -12.0, Math.toRadians(0.0))
        val parkingTwo = Pose2d(36.0, -12.0, Math.toRadians(0.0))

        val otw1Three = Pose2d(20.0, -13.0, Math.toRadians(0.0))
        val otw2Three = Pose2d(36.0, -12.0, Math.toRadians(0.0))
        val parkingThree = Pose2d(58.5, -12.0, Math.toRadians(0.0))


        var trajectoryToJunction = TrajectoryBuilder(rightHome, Math.toRadians(180.0), combinedConstraints)
            .splineToSplineHeading(otwJunction, Math.toRadians(80.0))
            .splineToSplineHeading(inFrontOfJunction, Math.toRadians(60.0))
            .build();

        var trajectoryGoForward = TrajectoryBuilder(trajectoryToJunction.end(), trajectoryToJunction.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        var trajectoryGoBackward = TrajectoryBuilder(trajectoryGoForward.end(), trajectoryGoForward.end().heading, combinedConstraints)
            .back(6.0)
            .build();

        var trajectoryToParkingOne = TrajectoryBuilder(trajectoryGoBackward.end(), Math.toRadians(90.0), combinedConstraints)
            .splineToSplineHeading(parkingOne, Math.toRadians(90.0))
            .build();

        var trajectoryToParkingTwo = TrajectoryBuilder(trajectoryGoBackward.end(), Math.toRadians(90.0), combinedConstraints)
            .splineToSplineHeading(otwTwo, Math.toRadians(10.0))
            .splineToSplineHeading(parkingTwo, Math.toRadians(0.0))
            .build();

        var trajectoryToParkingThree = TrajectoryBuilder(trajectoryGoBackward.end(), Math.toRadians(90.0), combinedConstraints)
            .splineToSplineHeading(otw1Three, Math.toRadians(10.0))
            .splineToSplineHeading(otw2Three, Math.toRadians(0.0))
            .splineToSplineHeading(parkingThree, Math.toRadians(0.0))
            .build();


        trajectories.add(trajectoryToJunction)
        trajectories.add(trajectoryGoForward)
        trajectories.add(trajectoryGoBackward)

        when (dest) {
            1 -> trajectories.add(trajectoryToParkingOne)
            2 -> trajectories.add(trajectoryToParkingTwo)
            3 -> trajectories.add(trajectoryToParkingThree)
        }


        return trajectories
    }


}