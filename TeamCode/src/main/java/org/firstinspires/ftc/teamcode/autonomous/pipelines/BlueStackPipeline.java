package org.firstinspires.ftc.teamcode.autonomous.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class BlueStackPipeline extends OpenCvPipeline
{
    /**
     * Define and initialize the robot's Telemetry
     */
    private final Telemetry telemetry;
    public BlueStackPipeline(Telemetry telemetry, double factor, double offset) {
        this.telemetry = telemetry;
        this.FACTOR = factor;
        this.CENTER_OFFSET = offset;
    }


    /**
     * Define the position of the junction
     * relative to the center of the image.
     */
    private volatile double displacement = 0;

    /**
     * Scaling FACTOR to map the camera's displacement to the robot's displacement
     */
    private double FACTOR;

    // The possible error from the camera to the robot's claw
    private double CENTER_OFFSET;

    // Scaling FACTOR (used to perform the operations faster)
    final double SCALE = 0.5;


    /**
     * Values used to perform color analysis
     */
    static final Scalar BLACK = new Scalar(0,0,0);
    static final Scalar BLUE = new Scalar(0,0,255);
    static final int BLUE_HUE = 120;
    // The brightness threshold is used to filter very dark or bright colors
    static final double BRIGHTNESS_THR = 0.2;


    /**
     * Matrix used to store the Mask that shows only yellow values
     */
    Mat mask = new Mat();




    /**
     * Convert the RGB region of the image to the HSV color scale,
     * mask the yellow colors, and blur the image.
     *
     * @param    input   the input image from the camera in RGB color scale
     */
    public void maskYellow(Mat input) {
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mask, new Scalar(BLUE_HUE -15, BRIGHTNESS_THR*255, BRIGHTNESS_THR*255), new Scalar(BLUE_HUE +15, 255, 255), mask);

        Mat kernel = new Mat();

        Imgproc.erode(mask, mask, kernel);
        Imgproc.erode(mask, mask, kernel);
        Imgproc.erode(mask, mask, kernel);
        Imgproc.erode(mask, mask, kernel);

        Imgproc.dilate(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);

    }


    /**
     * Detect the position of the junction, draws a bounding box,
     * and update the displacement from the origin
     *
     * @param  input   the input image from the camera in RGB color scale
     */
    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * Scale the image, and mask the yellow values.
         */
        Mat scaledInput = new Mat();
        Imgproc.resize(input, scaledInput, new Size(0,0), SCALE, SCALE);
        maskYellow(scaledInput);


        /*
         * Find the bounding box, the center of the junction,
         * and compute the displacement from the robot's claw
         */
        int[] x = findRange(mask, 0);
        int[] y = findRange(mask, 1);

        int center = (int)Math.round(x[1]/ SCALE);

        displacement = Math.cbrt((center - input.width()*0.5)/((double)input.width()) - CENTER_OFFSET);

        telemetry.addData("Displacement: ", displacement*FACTOR);
        telemetry.update();


        /*
         * Draw the bounding box and a center line through the junction.
         */
        Imgproc.rectangle(
                input,
                new Point(Math.round(x[0]/ SCALE), Math.round(y[0]/ SCALE)),
                new Point(Math.round(x[2]/ SCALE), Math.round(y[2]/ SCALE)),
                BLUE,
                2);

        Imgproc.line(input, new Point(center, 0), new Point(center, input.height()), BLUE, 2);
        Imgproc.line(input, new Point(input.width()*(CENTER_OFFSET+0.5), 0), new Point(input.width()*(CENTER_OFFSET+0.5), input.height()),BLACK, 2);


        /*
         * Release matrices to clear memory
         */
        mask.release();
        scaledInput.release();


        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }



    /**
     * Find the range of the junction at any axis.
     *
     * @param   mask    the masked image to find the range from
     * @param   axis    the axis (x or y) to find the range around
     * @return  an array containing the start, center, and end of the range.
     */
    public int[] findRange(Mat mask, int axis) {
        int length = mask.cols();
        if (axis==1) length = mask.rows();

        int[] sum = new int[length];
        for(int i = 0; i<length; i++) {
            if (axis==1) sum[i] = (int)Core.sumElems(mask.row(i)).val[0];
            else sum[i] = (int)Core.sumElems(mask.col(i)).val[0];
        }


        int longest = 0;
        int current = 0;
        int end = 0;

        for(int i=0; i<sum.length; i++) {
            if(sum[i]>=1) current++;
            else current=0;

            if(current>longest) {
                longest = current;
                end = i;
            }
        }


        return new int[]{end-longest+1, end+1 - longest/2, end+1};

    }



    /**
     * Get the displacement (in inches) from the robot to the junction.
     *
     * @return  the displacement in inches
     */
    public double getDisplacement()
    {
        return displacement * FACTOR;
    }

}