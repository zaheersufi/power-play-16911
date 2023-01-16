package com.example.pathvisualizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints


object TrajectoryTwoCones {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(50.0, 30.0, 0.0, Math.toRadians(399.0969417327928),  Math.toRadians(60.0), 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    const val ROBOT_WIDTH = 16.5

    private val combinedConstraints = MecanumConstraints(driveConstraints, ROBOT_WIDTH)

    private val list = ArrayList<Trajectory>()



    fun createTrajectory(): ArrayList<Trajectory> {
//        Left(1)
        Right(3)


        return list
    }


    private fun Left(dest: Int) {
        val home = Pose2d(-36.0, -63.0, Math.toRadians(90.0))

        val midJunction = Pose2d(-33.0, -24.0, Math.toRadians(-.0))

        val stack = Pose2d(-62.0, -12.0, Math.toRadians(180.0))

        val highJunction = Pose2d(-31.5, -7.5, Math.toRadians(45.0))

        val parkingOne = Pose2d(-59.0, -13.0, Math.toRadians(180.0))
        val parkingTwo = Pose2d(-36.0, -12.0, Math.toRadians(180.0))
        val parkingThree = Pose2d(-13.25, -12.0, Math.toRadians(180.0))

        /*
        1. Go to mid and raise lift while moving
        2. Go forward
        3. Lower lift and drop
        4. Go for another cone
        5. Go to high and raise lift while moving
        6. Go forward
        7. Lower lift and drop
        8. Park
        PATH TIME: 15s
         */


        var trajectoryToMid= TrajectoryBuilder(home, Math.toRadians(80.0), combinedConstraints)
            .splineToSplineHeading(midJunction, Math.toRadians(75.0))
            .build();

        var trajectoryMidForward = TrajectoryBuilder(trajectoryToMid.end(), trajectoryToMid.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        var trajectoryMidBackward = TrajectoryBuilder(trajectoryMidForward.end(), trajectoryMidForward.end().heading, combinedConstraints)
            .back(6.0)
            .build();

        var trajectoryToStack = TrajectoryBuilder(trajectoryMidBackward.end(), Math.toRadians(80.0), combinedConstraints)
            .splineToSplineHeading(stack, Math.toRadians(173.0))
            .build();

        var trajectoryToHigh = TrajectoryBuilder(trajectoryToStack.end(), Math.toRadians(0.0), combinedConstraints)
            .splineToSplineHeading(highJunction, Math.toRadians(60.0))
            .build();

        var trajectoryHighForward = TrajectoryBuilder(trajectoryToHigh.end(), trajectoryToHigh.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        var trajectoryHighBackward = TrajectoryBuilder(trajectoryHighForward.end(), trajectoryHighForward.end().heading, combinedConstraints)
            .back(4.0)
            .build();

        var trajectoryToParkingOne = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-120.0), combinedConstraints)
            .splineToSplineHeading(parkingOne, Math.toRadians(175.0))
            .build();

        var trajectoryToParkingTwo = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-120.0), combinedConstraints)
        .splineToSplineHeading(parkingTwo, Math.toRadians(135.0))
            .build();

        var trajectoryToParkingThree = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-90.0), combinedConstraints)
            .splineToSplineHeading(parkingThree, Math.toRadians(0.0))
            .build();

        list.add(trajectoryToMid) //Raise lift
        list.add(trajectoryMidForward)
        list.add(trajectoryMidBackward)
        list.add(trajectoryToStack) // No need to lower (maybe a bit), then raise.
        list.add(trajectoryToHigh) // Raise a bit
        list.add(trajectoryHighForward)
        list.add(trajectoryHighBackward) // Lower halfway. Lower the rest during parking

        if(dest == 1) list.add(trajectoryToParkingOne)
        else if(dest == 2) list.add(trajectoryToParkingTwo)
        else if(dest == 3) list.add(trajectoryToParkingThree)

    }



    private fun Right(dest: Int) {
        val home = Pose2d(36.0, -63.0, Math.toRadians(90.0))

        val midJunction = Pose2d(33.0, -24.0, Math.toRadians(180.0))

        val stack = Pose2d(62.0, -12.0, Math.toRadians(0.0))

        val highJunction = Pose2d(31.5, -7.5, Math.toRadians(135.0))

        val parkingOne = Pose2d(13.25, -12.0, Math.toRadians(0.0))
        val parkingTwo = Pose2d(36.0, -12.0, Math.toRadians(0.0))
        val parkingThree = Pose2d(59.0, -13.0, Math.toRadians(0.0))

        /*
        1. Go to mid and raise lift while moving
        2. Go forward
        3. Lower lift and drop
        4. Go for another cone
        5. Go to high and raise lift while moving
        6. Go forward
        7. Lower lift and drop
        8. Park
        PATH TIME: 15s
         */


        var trajectoryToMid= TrajectoryBuilder(home, Math.toRadians(100.0), combinedConstraints)
            .splineToSplineHeading(midJunction, Math.toRadians(95.0))
            .build();

        var trajectoryMidForward = TrajectoryBuilder(trajectoryToMid.end(), trajectoryToMid.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        var trajectoryMidBackward = TrajectoryBuilder(trajectoryMidForward.end(), trajectoryMidForward.end().heading, combinedConstraints)
            .back(6.0)
            .build();

        var trajectoryToStack = TrajectoryBuilder(trajectoryMidBackward.end(), Math.toRadians(100.0), combinedConstraints)
            .splineToSplineHeading(stack, Math.toRadians(7.0))
            .build();

        var trajectoryToHigh = TrajectoryBuilder(trajectoryToStack.end(), Math.toRadians(180.0), combinedConstraints)
            .splineToSplineHeading(highJunction, Math.toRadians(120.0))
            .build();

        var trajectoryHighForward = TrajectoryBuilder(trajectoryToHigh.end(), trajectoryToHigh.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        var trajectoryHighBackward = TrajectoryBuilder(trajectoryHighForward.end(), trajectoryHighForward.end().heading, combinedConstraints)
            .back(4.0)
            .build();

        var trajectoryToParkingOne = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-90.0), combinedConstraints)
            .splineToSplineHeading(parkingOne, Math.toRadians(180.0))
            .build();

        var trajectoryToParkingTwo = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-60.0), combinedConstraints)
            .splineToSplineHeading(parkingTwo, Math.toRadians(45.0))
            .build();

        var trajectoryToParkingThree = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-60.0), combinedConstraints)
            .splineToSplineHeading(parkingThree, Math.toRadians(5.0))
            .build();

        list.add(trajectoryToMid) //Raise lift
        list.add(trajectoryMidForward)
        list.add(trajectoryMidBackward)
        list.add(trajectoryToStack) // No need to lower (maybe a bit), then raise.
        list.add(trajectoryToHigh) // Raise a bit
        list.add(trajectoryHighForward)
        list.add(trajectoryHighBackward) // Lower halfway. Lower the rest during parking

        if(dest == 1) list.add(trajectoryToParkingOne)
        else if(dest == 2) list.add(trajectoryToParkingTwo)
        else if(dest == 3) list.add(trajectoryToParkingThree)

    }


}