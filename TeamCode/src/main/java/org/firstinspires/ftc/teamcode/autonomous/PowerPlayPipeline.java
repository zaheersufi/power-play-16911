//  java -jar "C:\Users\lgran\Desktop\EOCV-Sim-3.4.2-all.jar"



import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;



public class PowerPlayPipeline extends OpenCvPipeline
{
    /**
    * Define and initialize the robot's Telemetry
    */
    private final Telemetry telemetry;
    public PowerPlayPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }



    /**
    * Enum to define the possible destinations for the robot
    */
    public enum Destination
    {
        LEFT,
        CENTER,
        RIGHT
    }
    Destination destination;


    /**
    * Color cosntants
    * (all values are in the CIELAB color scale.
    * See more: https://en.wikipedia.org/wiki/CIELAB_color_space)
    */
    static final Scalar RED = new Scalar(52, 58, 41);       // NOT the best (a lot of red things)
    static final Scalar BLUE = new Scalar(56, 17, -68);     // NOT the best (a lot of blue things)
    static final Scalar YELLOW = new Scalar(97, -22, 94);   // Okay -> junctions are yellow (?)

    static final Scalar BLACK = new Scalar(0, 0, 0);        // Pretty good
    static final Scalar GREEN = new Scalar(48, -53, 51);    // Pretty good
    static final Scalar PINK = new Scalar(63, 91, -58);     // Pretty good

    // Array with all the colors being used
    static final Scalar[] colors = new Scalar[]{RED,BLUE,GREEN,PINK,YELLOW,BLACK};
    // Labels for each color (what each color means)
    static final String[] color_labels = new String[]{"RED","BLUE","GREEN","PINK","YELLOW","BLACK"};


    /**
    * Core values that define the location and size of the region
    */
    static final Point REGION_TOPLEFT_POINT = new Point(10,20);  // x,y (top left corner of the window is (0,0))
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 20;

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
    Mat region = new Mat(), LabRegion = new Mat();  // the subregion for the image that we care about (the analysis will be done on here)
    Scalar average;         // the average/mean color of the region in CIELAB color scale
    double[] dists = new double[colors.length]; // array of all the distances from the average color to each actual color


    /**
    * Extract the region of interest from the original image
    * Convert the RGB region of the image to the CIELAB color scale
    *
    * @param    input   the input image from the camera in RGB color scale
    */
    @Override
    public void init(Mat input)
    {
        region = input.submat(new Rect(region_pointA, region_pointB));  // extract the desired subregion
        Imgproc.cvtColor(region, LabRegion, Imgproc.COLOR_RGB2Lab);     // convert the RGB region to CIELAB and save it in LabRegion
    }



    /**
    * Extract the region of interest from the origianl image
    * Convert the RGB region of the image to the CIELAB color scale
    *
    * @param  input   the input image from the camera in RGB color scale
    */
    @Override
    public Mat processFrame(Mat input)
    {
        /**
        * Compute the average CIELAB pixel value for the region and scale them.
        * (We scale the values because these are 8-byte images, and each values is between 0-255.
        * See more: https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html)
        */
        average = Core.mean(LabRegion);
        average.val[0] = (int)(average.val[0]/2.55);
        average.val[1] = (int)(average.val[1]-128);
        average.val[2] = (int)(average.val[2]-128);


        /**
        * Compute the distance from the average color to each actual color and then get the index of the smallest.
        * The smallest distance represents the color is closest to.
        * (See more: https://en.wikipedia.org/wiki/Color_difference)
        */
        for(int i=0; i<colors.length; i++) {
            // The distance is computed using 3-dimensional euclidean distance: Pythagorean formula.
            dists[i] = Math.pow(colors[i].val[0]- average.val[0],2)
                        + Math.pow(colors[i].val[1]- average.val[1],2)
                        + Math.pow(colors[i].val[2]- average.val[2],2);
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
        * Draw a rectangle showing the sample region on the screen.
        * Simply a visual aid. Serves no functional purpose.
        */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region_pointA, // First point which defines the rectangle
                region_pointB, // Second point which defines the rectangle
                new Scalar(255,0,0), // The color the rectangle is drawn in
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
    public Destination getDestination()
    {
        return destination;
    }

}