package org.firstinspires.ftc.teamcode.autonomous;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    int lastResult = 0;
    @Override
    public Mat processFrame(Mat input) {
        return null;
    }
    public int getLastResult()
    {
        return lastResult;
    }
}
