package org.firstinspires.ftc.teamcode.autonomous;//  java -jar "C:\Users\lgran\Desktop\EOCV-Sim-3.4.2-all.jar"


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;



public class PowerPlayPipeline_HSV extends OpenCvPipeline
{
    /**
    * Define and initialize the robot's Telemetry
    */
    private final Telemetry telemetry;
    public PowerPlayPipeline_HSV(Telemetry telemetry) {
        this.telemetry = telemetry;
    }



    /**
    * Enum to define the possible destinations for the robot
    */
    volatile int destination = 0;



    /**
    * Colors being used and their HUE values
    * (all values HUE values are in the HSV color scale.
    * See more: https://en.wikipedia.org/wiki/HSL_and_HSV)
    */
    // Labels for each color (what each color means)
    static final String[] color_labels = new String[]{"YELLOW","CYAN","MAGENTA"};

    static final Scalar YELLOW = new Scalar(255,255,0);
    static final Scalar CYAN = new Scalar(0,255,255);
    static final Scalar MAGENTA = new Scalar(255,0,255);
    // Array with all the colors being used in RGB
    static final Scalar[] RGB_COLORS = new Scalar[]{YELLOW, CYAN, MAGENTA};

    static final int YELLOW_HUE = 30;
    static final int CYAN_HUE = 90;
    static final int MAGENTA_HUE = 150;
    // Array with all the colors' hues being used
    static final int[] COLOR_HUES = new int[]{YELLOW_HUE, CYAN_HUE, MAGENTA_HUE};



    /**
    * Core values that define the location and size of the region
    */
    static final Point REGION_TOPLEFT_POINT = new Point(250,150);  // x,y (top left corner of the window is (0,0))
    static final int REGION_WIDTH = 30;
    static final int REGION_HEIGHT = 30;


    
    /**
    * Points which actually define the sample region rectangles, derived from above values
    * Example of how points A and B work to define a rectangle
    *
    *   ------------------------
    *   | (0,0) Point A        |
    *   |                      |
    *   |      Point B (70,50) |
    *   ------------------------
    *
    */
    Point region_pointA = new Point(REGION_TOPLEFT_POINT.x, REGION_TOPLEFT_POINT.y);
    Point region_pointB = new Point(REGION_TOPLEFT_POINT.x + REGION_WIDTH, REGION_TOPLEFT_POINT.y + REGION_HEIGHT);



    /**
    * Working variables
    */
    Mat region = new Mat(), hsvRegion = new Mat();  // the subregion for the image that we care about (the analysis will be done on here)
    Scalar average;                                 // the average/mean color of the region in HSV color scale
    double[] dists = new double[COLOR_HUES.length];     // array of all the distances from the average color to each actual color



    /**
    * Extract the region of interest from the original image
    * Convert the RGB region of the image to the HSV color scale
    *
    * @param    input   the input image from the camera in RGB color scale
    */
    @Override
    public void init(Mat input)
    {
        region = input.submat(new Rect(region_pointA, region_pointB));  // extract the desired subregion
        Imgproc.cvtColor(region, hsvRegion, Imgproc.COLOR_RGB2HSV);     // convert the RGB region to HSV and save it in HsvRegion
    }



    /**
    * Extract the region of interest from the origianl image
    * Convert the RGB region of the image to the HSV color scale
    *
    * @param  input   the input image from the camera in RGB color scale
    */
    @Override
    public Mat processFrame(Mat input)
    {
        /**
        * Compute the average HSV pixel value for the region.
        */
        average = Core.mean(hsvRegion);


        /**
        * Compute the distance from the average color to each actual color and then get the index of the smallest.
        * The smallest distance represents the color is closest to.
        * (See more: https://en.wikipedia.org/wiki/Color_difference)
        */
        for(int i = 0; i< COLOR_HUES.length; i++) {
            dists[i] = Math.abs(average.val[0] - COLOR_HUES[i]);
        }

        // Compare all distances to each other and determine the smallest.
        int idx = -1;
        for(int i=0; i< dists.length; i++) {
            boolean smaller = true;
            for (double dist : dists) {
                if (dists[i] > dist) {
                    smaller = false;
                    break;
                }
            }
            if (smaller){
                idx = i;
                break;
            }
        }


        /**
         * Show the closest color in the Telemetry.
         * If no color was determined, show a warning.
         */
        if (idx<0) telemetry.addData("WARNING: ", "undecided color"); // Make sure that a distance was determined.
        else telemetry.addData("COLOR: ", color_labels[idx]);

        telemetry.addData("Avgs: ", average); // Show the average color, just in case.


        /**
         * Update the destination depending on the color.
         */
        if (idx==0) destination = 0;         // LEFT if Yellow
        else if (idx==1) destination = 0;  // CENTER if Cyan
        else if (idx==2) destination = 0;   // RIGHT if Magenta


        /**
         * Draw a rectangle showing the sample region on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region_pointA, // First point which defines the rectangle
                region_pointB, // Second point which defines the rectangle
                RGB_COLORS[idx], // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        /**
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }



    /**
    * Get the destination/position the element indicates.
    *
    * @return  the robot's destination as indicated by hte previous analysis
    */
    public int getDestination()
    {
        return destination;
    }

}