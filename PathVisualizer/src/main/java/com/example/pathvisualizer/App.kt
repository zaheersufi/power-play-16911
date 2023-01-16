package com.example.pathvisualizer

import com.acmerobotics.roadrunner.trajectory.Trajectory
import javafx.animation.KeyFrame
import javafx.animation.Timeline
import javafx.application.Application
import javafx.scene.Group
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.image.Image
import javafx.scene.layout.StackPane
import javafx.scene.paint.Color
import javafx.scene.shape.Line
import javafx.scene.shape.Rectangle

import javafx.stage.Stage
import javafx.util.Duration

class App : Application() {

    private lateinit var fieldImage: Image
    private lateinit var stage: Stage
    private lateinit var gc:  GraphicsContext

    private var BOTS = listOf<Bot>(Bot(TrajectoryOneConeMid().left(2)),
        Bot(TrajectoryTwoCones().left(2)),
        Bot(TrajectoryTwoCones().right(2)))

    companion object {
        var WIDTH = 0.0
        var HEIGHT = 0.0
    }

    override fun start(stage: Stage?) {
        for (bot in BOTS) {
            if (bot.startTime.isNaN()) bot.resetTimer()
        }

        this.stage = stage!!
        fieldImage = Image("file:PathVisualizer/src/main/java/com/example/pathvisualizer/powerPlayField.png")
        println(fieldImage)
        val root = Group()

        WIDTH = fieldImage.width
        HEIGHT = fieldImage.height
        GraphicsUtil.pixelsPerInch = WIDTH / GraphicsUtil.FIELD_WIDTH
        GraphicsUtil.halfFieldPixels = WIDTH / 2.0

        val canvas = Canvas(WIDTH, HEIGHT)
        gc = canvas.graphicsContext2D
        GraphicsUtil.gc = gc
        gc.drawImage(fieldImage, 0.0, 0.0)
        gc.lineWidth = GraphicsUtil.LINE_THICKNESS

        val t1 = Timeline(KeyFrame(Duration.millis(10.0), { run(gc) }))
        t1.cycleCount = Timeline.INDEFINITE

        stage.scene = Scene(
            StackPane(
                root
            )
        )

        root.children.add(canvas)

        for (bot in BOTS) {
            bot.setGraphics(root)
        }

        stage.title = "PathVisualizer"
        stage.isResizable = false

        var i = 1
        for (bot in BOTS) {
            println("Bot $i duration: ${"%.2f".format(bot.totalDuration)}")
            i++
        }

        stage.show()
        t1.play()
    }

    fun run(gc: GraphicsContext) {
        for (bot in BOTS) {
            bot.drawTrajectory()

            var finished = true
            for (b in BOTS) {
                finished = finished && b.isFinished
            }
            if (finished) {
                for (b in BOTS) {
                    b.reset()
                }
                return
            }
        }

        var maxTimeIndex = 0
        for (i in 0 until BOTS.size) {
            if (BOTS[i].totalDuration > BOTS[maxTimeIndex].totalDuration)
                maxTimeIndex = i
        }

        stage.title = "Total Time: ${"%.1f".format(BOTS[maxTimeIndex].totalDuration)} - Time : ${"%.1f".format(BOTS[maxTimeIndex].time - BOTS[maxTimeIndex].startTime)}"
    }

}

fun main(args: Array<String>) {
    Application.launch(App::class.java, *args)
}