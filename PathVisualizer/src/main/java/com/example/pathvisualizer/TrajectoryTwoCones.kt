package com.example.pathvisualizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder

class TrajectoryTwoCones : TrajectoryTemplate() {

    override fun left(dest: Int): ArrayList<Trajectory> {
        val trajectories = ArrayList<Trajectory>()


        val home = Pose2d(-36.0, -63.0, Math.toRadians(90.0));

        val midJunction = Pose2d(-33.0, -24.0, Math.toRadians(-.0));

        val stack = Pose2d(-62.0, -12.0, Math.toRadians(180.0));

        val highJunction = Pose2d(-31.5, -7.5, Math.toRadians(45.0));

        val parkingOne = Pose2d(-59.0, -13.0, Math.toRadians(180.0));
        val parkingTwo = Pose2d(-36.0, -12.0, Math.toRadians(180.0));
        val parkingThree = Pose2d(-13.25, -12.0, Math.toRadians(180.0));

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


        val trajectoryToMid= TrajectoryBuilder(home, Math.toRadians(80.0), combinedConstraints)
            .splineToSplineHeading(midJunction, Math.toRadians(75.0))
            .build();

        val trajectoryMidForward = TrajectoryBuilder(trajectoryToMid.end(), trajectoryToMid.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        val trajectoryMidBackward = TrajectoryBuilder(trajectoryMidForward.end(), trajectoryMidForward.end().heading, combinedConstraints)
            .back(6.0)
            .build();

        val trajectoryToStack = TrajectoryBuilder(trajectoryMidBackward.end(), Math.toRadians(80.0), combinedConstraints)
            .splineToSplineHeading(stack, Math.toRadians(173.0))
            .build();

        val trajectoryToHigh = TrajectoryBuilder(trajectoryToStack.end(), Math.toRadians(0.0), combinedConstraints)
            .splineToSplineHeading(highJunction, Math.toRadians(60.0))
            .build();

        val trajectoryHighForward = TrajectoryBuilder(trajectoryToHigh.end(), trajectoryToHigh.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        val trajectoryHighBackward = TrajectoryBuilder(trajectoryHighForward.end(), trajectoryHighForward.end().heading, combinedConstraints)
            .back(4.0)
            .build();

        val trajectoryToParkingOne = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-120.0), combinedConstraints)
            .splineToSplineHeading(parkingOne, Math.toRadians(175.0))
            .build();

        val trajectoryToParkingTwo = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-120.0), combinedConstraints)
            .splineToSplineHeading(parkingTwo, Math.toRadians(135.0))
            .build();

        val trajectoryToParkingThree = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-90.0), combinedConstraints)
            .splineToSplineHeading(parkingThree, Math.toRadians(0.0))
            .build();

        trajectories.add(trajectoryToMid) //Raise lift
        trajectories.add(trajectoryMidForward)
        trajectories.add(trajectoryMidBackward)
        trajectories.add(trajectoryToStack) // No need to lower (maybe a bit), then raise.
        trajectories.add(trajectoryToHigh) // Raise a bit
        trajectories.add(trajectoryHighForward)
        trajectories.add(trajectoryHighBackward) // Lower halfway. Lower the rest during parking


        when (dest) {
            1 -> trajectories.add(trajectoryToParkingOne)
            2 -> trajectories.add(trajectoryToParkingTwo)
            3 -> trajectories.add(trajectoryToParkingThree)
        }

        return trajectories
    }

    override fun right(dest: Int): ArrayList<Trajectory> {
        val trajectories = ArrayList<Trajectory>()


        val home = Pose2d(36.0, -63.0, Math.toRadians(90.0));

        val midJunction = Pose2d(33.0, -24.0, Math.toRadians(180.0));

        val stack = Pose2d(62.0, -12.0, Math.toRadians(0.0));

        val highJunction = Pose2d(31.5, -7.5, Math.toRadians(135.0));

        val parkingOne = Pose2d(13.25, -12.0, Math.toRadians(0.0));
        val parkingTwo = Pose2d(36.0, -12.0, Math.toRadians(0.0));
        val parkingThree = Pose2d(59.0, -13.0, Math.toRadians(0.0));

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


        val trajectoryToMid= TrajectoryBuilder(home, Math.toRadians(100.0), combinedConstraints)
            .splineToSplineHeading(midJunction, Math.toRadians(95.0))
            .build();

        val trajectoryMidForward = TrajectoryBuilder(trajectoryToMid.end(), trajectoryToMid.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        val trajectoryMidBackward = TrajectoryBuilder(trajectoryMidForward.end(), trajectoryMidForward.end().heading, combinedConstraints)
            .back(6.0)
            .build();

        val trajectoryToStack = TrajectoryBuilder(trajectoryMidBackward.end(), Math.toRadians(100.0), combinedConstraints)
            .splineToSplineHeading(stack, Math.toRadians(7.0))
            .build();

        val trajectoryToHigh = TrajectoryBuilder(trajectoryToStack.end(), Math.toRadians(180.0), combinedConstraints)
            .splineToSplineHeading(highJunction, Math.toRadians(120.0))
            .build();

        val trajectoryHighForward = TrajectoryBuilder(trajectoryToHigh.end(), trajectoryToHigh.end().heading, combinedConstraints)
            .forward(4.0)
            .build();

        val trajectoryHighBackward = TrajectoryBuilder(trajectoryHighForward.end(), trajectoryHighForward.end().heading, combinedConstraints)
            .back(4.0)
            .build();

        val trajectoryToParkingOne = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-90.0), combinedConstraints)
            .splineToSplineHeading(parkingOne, Math.toRadians(180.0))
            .build();

        val trajectoryToParkingTwo = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-60.0), combinedConstraints)
            .splineToSplineHeading(parkingTwo, Math.toRadians(45.0))
            .build();

        val trajectoryToParkingThree = TrajectoryBuilder(trajectoryHighBackward.end(), Math.toRadians(-60.0), combinedConstraints)
            .splineToSplineHeading(parkingThree, Math.toRadians(5.0))
            .build();

        trajectories.add(trajectoryToMid) //Raise lift
        trajectories.add(trajectoryMidForward)
        trajectories.add(trajectoryMidBackward)
        trajectories.add(trajectoryToStack) // No need to lower (maybe a bit), then raise.
        trajectories.add(trajectoryToHigh) // Raise a bit
        trajectories.add(trajectoryHighForward)
        trajectories.add(trajectoryHighBackward) // Lower halfway. Lower the rest during parking


        when (dest) {
            1 -> trajectories.add(trajectoryToParkingOne)
            2 -> trajectories.add(trajectoryToParkingTwo)
            3 -> trajectories.add(trajectoryToParkingThree)
        }


        return trajectories
    }

}