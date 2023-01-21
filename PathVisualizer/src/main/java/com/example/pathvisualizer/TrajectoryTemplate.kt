package com.example.pathvisualizer

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints

abstract class TrajectoryTemplate {
    companion object {
        const val robotWidth = 15.0
        const val robotLength = 16.5
    }

    private val driveConstraints = DriveConstraints(57.0, 45.0, 0.0, Math.toRadians(399.0969417327928),  Math.toRadians(60.0), 0.0)
    val combinedConstraints = MecanumConstraints(driveConstraints, (robotWidth+ robotLength)/2)


    abstract fun left(dest: Int): ArrayList<Trajectory>
    abstract fun right(dest: Int): ArrayList<Trajectory>

}