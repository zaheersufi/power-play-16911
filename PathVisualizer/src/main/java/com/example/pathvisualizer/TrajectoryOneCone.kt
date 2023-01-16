import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints


object TrajectoryOneCone {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(50.0, 30.0, 0.0, Math.toRadians(399.0969417327928),  Math.toRadians(60.0), 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    const val ROBOT_WIDTH = 16.5

    private val combinedConstraints = MecanumConstraints(driveConstraints, ROBOT_WIDTH)

    private val list = ArrayList<Trajectory>()



    fun createTrajectory(): ArrayList<Trajectory> {
        highAllianceLeft(1)
//        highAllianceRight(3)

//        midAllianceLeft(3)
//        midAllianceRight(3)


        return list
    }


    fun highAllianceLeft(dest: Int) {
        val leftHome = Pose2d(36.0, 62.5, Math.toRadians(-90.0))

        val otwJunction = Pose2d(10.0, 47.0, Math.toRadians(-90.0))
        val inFrontOfJunction = Pose2d(8.0, 24.0, Math.toRadians(-180.0))

        val otw1One = Pose2d(20.0, 13.0, Math.toRadians(-70.0))
        val otw2One = Pose2d(36.0, 12.0, Math.toRadians(0.0))
        val parkingOne = Pose2d(58.5, 12.0, Math.toRadians(0.0))

        val otwTwo = Pose2d(20.0, 12.0, Math.toRadians(-90.0))
        val parkingTwo = Pose2d(36.0, 12.0, Math.toRadians(0.0))

        val parkingThree = Pose2d(13.25, 12.0, Math.toRadians(0.0))


        var trajectoryToJunction = TrajectoryBuilder(leftHome, Math.toRadians(-180.0), combinedConstraints)
            .splineToSplineHeading(otwJunction, Math.toRadians(-90.0))
            .splineToSplineHeading(inFrontOfJunction, Math.toRadians(-120.0))
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

        list.add(trajectoryToJunction)
        list.add(trajectoryGoForward)
        list.add(trajectoryGoBackward)

        if(dest == 1) list.add(trajectoryToParkingOne)
        else if(dest == 2) list.add(trajectoryToParkingTwo)
        else if(dest == 3) list.add(trajectoryToParkingThree)

    }



    fun highAllianceRight(dest: Int) {
        val rightHome = Pose2d(36.0, -62.5, Math.toRadians(90.0))

        val otwJunction = Pose2d(10.0, -45.0, Math.toRadians(90.0))
        val inFrontOfJunction = Pose2d(8.0, -24.0, Math.toRadians(180.0))

        val parkingOne = Pose2d(13.25, -12.0, Math.toRadians(0.0))

        val otwTwo = Pose2d(20.0, -12.0, Math.toRadians(90.0))
        val parkingTwo = Pose2d(36.0, -12.0, Math.toRadians(0.0))

        val otw1Three = Pose2d(20.0, -13.0, Math.toRadians(70.0))
        val otw2Three = Pose2d(36.0, -12.0, Math.toRadians(0.0))
        val parkingThree = Pose2d(58.5, -12.0, Math.toRadians(0.0))


        var trajectoryToJunction = TrajectoryBuilder(rightHome, Math.toRadians(180.0), combinedConstraints)
            .splineToSplineHeading(otwJunction, Math.toRadians(90.0))
            .splineToSplineHeading(inFrontOfJunction, Math.toRadians(120.0))
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


        list.add(trajectoryToJunction)
        list.add(trajectoryGoForward)
        list.add(trajectoryGoBackward)

        if(dest == 1) list.add(trajectoryToParkingOne)
        else if(dest == 2) list.add(trajectoryToParkingTwo)
        else if(dest == 3) list.add(trajectoryToParkingThree)

    }



    fun midAllianceLeft(dest: Int) {
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


        list.add(trajectoryToJunction)
        list.add(trajectoryGoForward)
        list.add(trajectoryGoBackward)

        if(dest == 1) list.add(trajectoryToParkingOne)
        else if(dest == 2) list.add(trajectoryToParkingTwo)
        else if(dest == 3) list.add(trajectoryToParkingThree)

    }



    fun midAllianceRight(dest: Int) {
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

        list.add(trajectoryToJunction)
        list.add(trajectoryGoForward)
        list.add(trajectoryGoBackward)

        if(dest == 1) list.add(trajectoryToParkingOne)
        else if(dest == 2) list.add(trajectoryToParkingTwo)
        else if(dest == 3) list.add(trajectoryToParkingThree)

    }


}