package com.example.pathvisualizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import javafx.scene.canvas.GraphicsContext

import javafx.scene.paint.Color
import javafx.scene.shape.Line
import javafx.scene.shape.Rectangle


object GraphicsUtil {
    val DEFUALT_RESOLUTION = 2.0 // inches

    val FIELD_WIDTH = 144.0 // 12'

    val ROBOT_WIDTH = TrajectoryTemplate.robotWidth

    val LINE_THICKNESS = 3.0

    val PATH_COLOR = Color.BLACK
    val ROBOT_COLOR = Color.MAROON
    val ROBOT_VECTOR_COLOR = Color.BLUE
    val END_BOX_COLOR = Color.GREEN

    lateinit var gc: GraphicsContext

    fun setColor(color: Color) {
        gc.stroke = color
        gc.fill = color
    }

    fun drawSampledPath(path: Path) {
        setColor(PATH_COLOR)
        val samples = Math.ceil(path.length() / DEFUALT_RESOLUTION).toInt()
        val points = Array(samples) { Vector2d() }
        val dx = path.length() / (samples - 1).toDouble()
        for (i in 0 until samples) {
            val displacement = i * dx
            val pose = path[displacement]
            points[i] = pose.vec()
        }
        strokePolyline(points)
    }

    fun strokePolyline(points: Array<Vector2d>) {
        val pixels = points.map { it.toPixel }
        gc.strokePolyline(pixels.map { it.x }.toDoubleArray(), pixels.map { it.y }.toDoubleArray(), points.size)

    }

    fun updateRobotVector(vector: Line, pose2d: Pose2d) {
        val point1 = pose2d.vec()
        val v = pose2d.headingVec() * ROBOT_WIDTH / 2.0
        val point2 = point1 + v

        vector.startX = point1.toPixel.x
        vector.startY = point1.toPixel.y
        vector.endX = point2.toPixel.x
        vector.endY = point2.toPixel.y
        vector.fill = ROBOT_COLOR
        vector.strokeWidth = LINE_THICKNESS
        vector.stroke = ROBOT_VECTOR_COLOR
        vector.opacity = 0.6
    }

    fun updateRobotRect(rectangle: Rectangle, pose2d: Pose2d, color: Color, opacity: Double) {
        val pix_w = ROBOT_WIDTH * pixelsPerInch

        rectangle.width = pix_w
        rectangle.height = pix_w

        val center_pix = pose2d.vec().toPixel
        rectangle.x = center_pix.x - pix_w / 2.0
        rectangle.y = center_pix.y - pix_w / 2.0
        rectangle.fill = color
        rectangle.opacity = opacity
        rectangle.rotate = Math.toDegrees(-pose2d.heading)
    }

    var pixelsPerInch = 0.0
    var halfFieldPixels = 0.0
}


val Vector2d.toPixel
    get() = Vector2d(
        -y * GraphicsUtil.pixelsPerInch + GraphicsUtil.halfFieldPixels,
        -x * GraphicsUtil.pixelsPerInch + GraphicsUtil.halfFieldPixels
    )