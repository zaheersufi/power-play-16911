package com.example.pathvisualizer

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints

abstract class TrajectoryTemplate {
    companion object {
        val robotWidth = 16.5
    }

    private val driveConstraints = DriveConstraints(45.0, 30.0, 0.0, Math.toRadians(399.0969417327928),  Math.toRadians(60.0), 0.0)
    val combinedConstraints = MecanumConstraints(driveConstraints, robotWidth)


    abstract fun left(dest: Int): ArrayList<Trajectory>
    abstract fun right(dest: Int): ArrayList<Trajectory>

}