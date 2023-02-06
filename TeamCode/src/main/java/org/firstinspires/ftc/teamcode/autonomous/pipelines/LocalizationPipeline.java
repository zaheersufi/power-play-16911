package org.firstinspires.ftc.teamcode.autonomous.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class LocalizationPipeline extends OpenCvPipeline {
    private final Telemetry telemetry;
    public LocalizationPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    static final Scalar YELLOW = new Scalar(255,255,0);
    static final double BRIGHTNESS_THR = 0.4;
    private static final int YELLOW_HUE = 30;

    private final double centerline = 0.5;
    private final double top = 0.8;
    private final double bottom = 0.2;


    private int line;
    private final int thr = 20;


    private boolean isCentered = false;



    /**
     * The processFrame method processes a video frame and returns the processed frame
     * with additional information added, such as lines and yellow object detection results.
     *
     * @param input the original video frame to be processed
     * @return the processed video frame with additional information
     */
    @Override
    public Mat processFrame(Mat input) {
        line = (int)(input.width()*centerline);


        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);


        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(YELLOW_HUE-15, BRIGHTNESS_THR*255, BRIGHTNESS_THR*255), new Scalar(YELLOW_HUE+15, 255, 255), mask);


        Mat kernel = new Mat();
        Imgproc.erode(mask, mask, kernel);
        Imgproc.erode(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);


        isCentered = computeCentered(mask);


        Imgproc.line(mask, new Point(line-thr, 0), new Point(line-thr, mask.height()),YELLOW, 2);
        Imgproc.line(mask, new Point(line+thr, 0), new Point(line+thr, mask.height()),YELLOW, 2);

        Imgproc.line(mask, new Point(0, mask.height()*(1-top)), new Point(mask.width(),mask.height()*(1-top)),YELLOW, 2);
        Imgproc.line(mask, new Point(0, mask.height()*(1-bottom)), new Point(mask.width(), mask.height()*(1-bottom)),YELLOW, 2);


        telemetry.addData("Center", isObjectCentered());
        telemetry.update();


        return mask;
    }



    public boolean computeCentered(Mat mask){
        int countX = 0;
        for (int x = line - thr; x <= line + thr; x++) {
            int countY = 0;
            for (int y = (int)((1-top)*mask.height()); y < (int)((1-bottom)*mask.height()); y++) {
                double val = mask.get(y, x)[0];
                if (val>0) {
                    countY++;
                }
            }
            if(countY > mask.height()*(top-bottom)*0.3) countX++;
            if(countX>thr) return true;
        }
        return false;
    }



    public boolean isObjectCentered() {
        return isCentered;
    }
}
