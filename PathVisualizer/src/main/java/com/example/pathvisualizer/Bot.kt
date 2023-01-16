package com.example.pathvisualizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import javafx.scene.Group
import javafx.scene.shape.Line
import javafx.scene.shape.Rectangle

class Bot (traj: ArrayList<Trajectory>) {
    private val robotRect = Rectangle(100.0, 100.0, 10.0, 10.0)
    private val startRect = Rectangle(100.0, 100.0, 10.0, 10.0)
    private val endRect = Rectangle(100.0, 100.0, 10.0, 10.0)
    private val vector = Line(100.0,100.0,10.0,10.0)

    private var trajectories: ArrayList<Trajectory> = traj
    private var currentIndex: Int = 0

    private val trajectoryDurations = trajectories.map { it.duration() }
    private val numberOfTrajectories = trajectories.size
    val totalDuration = trajectoryDurations.sum()


    var startTime = Double.NaN
    var time = Double.NaN
    var isFinished = false


    fun setGraphics (root: Group) {
        root.children.addAll(startRect, endRect, robotRect, vector)
    }

    fun reset() {
        isFinished = false
        currentIndex = 0
        resetTimer()
    }

    fun resetTimer() {
        startTime = Clock.seconds
    }


    fun drawTrajectory() {
        if (isFinished) {
            return
        }

        val trajectory = trajectories[currentIndex]

        var x = 0.0
        for (i in 0 until currentIndex)
            x += trajectoryDurations[i]
        val prevDurations: Double = x

        time = Clock.seconds
        val profileTime = time - startTime - prevDurations
        val duration = trajectoryDurations[currentIndex]


        if (profileTime >= duration) {
            currentIndex++
            if (currentIndex >= numberOfTrajectories){
                isFinished = true
            }
        }

        trajectories.forEach{ GraphicsUtil.drawSampledPath(it.path) }

        updatePath(trajectory[profileTime])
    }

    fun updatePath(current: Pose2d) {
        GraphicsUtil.updateRobotRect(startRect, trajectories.first().start(), GraphicsUtil.END_BOX_COLOR, 0.5)
        GraphicsUtil.updateRobotRect(endRect, trajectories.last().end(), GraphicsUtil.END_BOX_COLOR, 0.5)

        GraphicsUtil.updateRobotRect(robotRect, current, GraphicsUtil.ROBOT_COLOR, 0.75)
        GraphicsUtil.updateRobotVector(vector, current)
    }
}