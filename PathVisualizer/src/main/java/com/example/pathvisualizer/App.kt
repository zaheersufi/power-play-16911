package com.example.pathvisualizer

import javafx.animation.KeyFrame
import javafx.animation.Timeline
import javafx.application.Application
import javafx.scene.Group
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.image.Image
import javafx.scene.layout.StackPane
import javafx.stage.Stage
import javafx.util.Duration


class App : Application() {

    private lateinit var fieldImage: Image
    private lateinit var stage: Stage
    private lateinit var gc:  GraphicsContext

    private var bots = listOf(
//        Bot(TrajectoryTwoCones().left(2), false),
//        Bot(TrajectoryTwoCones().right(2), true),
//        Bot(TrajectoryTwoCones().left(2), true),
//        Bot(TrajectoryTwoCones().right(2), false),
          Bot(TrajectoryMultiple().left(3), false),
          Bot(TrajectoryMultiple().left(1), true)
    )

    companion object {
        var WIDTH = 0.0
        var HEIGHT = 0.0
    }

    override fun start(stage: Stage?) {
        for (bot in bots) {
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

        val field = Canvas(WIDTH, HEIGHT)
        field.graphicsContext2D.drawImage(fieldImage, 0.0, 0.0)
        stage.scene = Scene(StackPane(root))
        root.children.add(field)

        for (bot in bots) {
            bot.setGraphics(root)
        }

        stage.title = "PathVisualizer"
        stage.isResizable = false
        stage.show()

        var i = 1
        for (bot in bots) {
            println("Bot $i duration: ${"%.2f".format(bot.totalDuration)}")
            print(bot)
            i++
        }

        val t1 = Timeline(KeyFrame(Duration.millis(10.0), { run() }))
        t1.cycleCount = Timeline.INDEFINITE
        t1.play()
    }

    fun run() {
        for (bot in bots) {
            bot.drawTrajectory()

            var finished = true
            for (b in bots) {
                finished = finished && b.isFinished
            }
            if (finished) {
                for (b in bots) {
                    b.reset()
                }
                return
            }
        }

        var maxTimeIndex = 0
        for (i in 0 until bots.size) {
            if (bots[i].totalDuration > bots[maxTimeIndex].totalDuration)
                maxTimeIndex = i
        }

        stage.title = "Total Time: ${"%.1f".format(bots[maxTimeIndex].totalDuration)} - Time : ${"%.1f".format(bots[maxTimeIndex].time - bots[maxTimeIndex].startTime)}"
    }

}

fun main(args: Array<String>) {
    Application.launch(App::class.java, *args)
}