package com.example.pathvisualizer

import javafx.animation.KeyFrame
import javafx.animation.Timeline
import javafx.application.Application
import javafx.event.ActionEvent
import javafx.event.EventHandler
import javafx.scene.Group
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.image.Image
import javafx.scene.layout.StackPane
import javafx.scene.paint.Color
import javafx.scene.shape.Rectangle

import javafx.stage.Stage
import javafx.util.Duration

class App : Application() {
    private val robotRect = Rectangle(100.0, 100.0, 10.0, 10.0)
    private val startRect = Rectangle(100.0, 100.0, 10.0, 10.0)
    private val endRect = Rectangle(100.0, 100.0, 10.0, 10.0)

    private var startTime = Double.NaN
//    private val trajectories = TrajectoryOneCone.createTrajectory()
    private val trajectories = TrajectoryTwoCones.createTrajectory()

    private lateinit var fieldImage: Image
    private lateinit var stage: Stage


    var activeTrajectoryIndex = 0
    val trajectoryDurations = trajectories.map { it.duration() }
    val totalDuration = trajectoryDurations.sum()
    val numberOfTrajectories = trajectories.size

    companion object {
        var WIDTH = 0.0
        var HEIGHT = 0.0
    }

    override fun start(stage: Stage?) {
        this.stage = stage!!
        fieldImage = Image("file:PathVisualizer/src/main/java/com/example/pathvisualizer/powerPlayField.png")
        println(fieldImage)
        val root = Group()

        WIDTH = fieldImage.width
        HEIGHT = fieldImage.height
        GraphicsUtil.pixelsPerInch = WIDTH / GraphicsUtil.FIELD_WIDTH
        GraphicsUtil.halfFieldPixels = WIDTH / 2.0

        val canvas = Canvas(WIDTH, HEIGHT)
        val gc = canvas.graphicsContext2D
        val t1 = Timeline(KeyFrame(Duration.millis(10.0), EventHandler<ActionEvent> { run(gc) }))
        t1.cycleCount = Timeline.INDEFINITE

        stage.scene = Scene(
            StackPane(
                root
            )
        )

        root.children.addAll(canvas, startRect, endRect, robotRect)

        stage.title = "PathVisualizer"
        stage.isResizable = false

        println("duration $totalDuration")

        stage.show()
        t1.play()
    }

    fun run(gc: GraphicsContext) {
        if (startTime.isNaN())
            startTime = Clock.seconds

        GraphicsUtil.gc = gc
        gc.drawImage(fieldImage, 0.0, 0.0)

        gc.lineWidth = GraphicsUtil.LINE_THICKNESS

        gc.globalAlpha = 0.5
        GraphicsUtil.setColor(Color.RED)
        gc.globalAlpha = 1.0

        val trajectory = trajectories[activeTrajectoryIndex]

        var x = 0.0
        for (i in 0 until activeTrajectoryIndex)
            x += trajectoryDurations[i]
        val prevDurations: Double = x

        val time = Clock.seconds
        val profileTime = time - startTime - prevDurations
        val duration = trajectoryDurations[activeTrajectoryIndex]

        val start = trajectories.first().start()
        val end = trajectories.last().end()
        val current = trajectory[profileTime]

        if (profileTime >= duration) {
            activeTrajectoryIndex++
            if(activeTrajectoryIndex >= numberOfTrajectories) {
                activeTrajectoryIndex = 0
                startTime = time
            }
        }

        trajectories.forEach{ GraphicsUtil.drawSampledPath(it.path) }

        GraphicsUtil.updateRobotRect(startRect, start, GraphicsUtil.END_BOX_COLOR, 0.5)
        GraphicsUtil.updateRobotRect(endRect, end, GraphicsUtil.END_BOX_COLOR, 0.5)

        GraphicsUtil.updateRobotRect(robotRect, current, GraphicsUtil.ROBOT_COLOR, 0.75)
        GraphicsUtil.drawRobotVector(current)

        stage.title = "Total Time: ${"%.2f".format(totalDuration)} - Current Time : ${"%.2f".format(time - startTime)}"
    }
}

fun main(args: Array<String>) {
    Application.launch(App::class.java, *args)
}