package org.firstinspires.ftc.teamcode.autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.HashMap;

public class qrScanner extends OpenCvPipeline {

    //set up robot's telemetry
    private final Telemetry telemetry;

    public qrScanner(Telemetry telemetry) {this.telemetry = telemetry;}

    /*  Create a qrScanner object to decode the qr code, a new Mat object called
    points to store the points on the image where the detectAndDecode method returns as well as
    define a string value that is to be used for destinations as the name suggests */
    QRCodeDetector qrcode = new QRCodeDetector();
    Mat points = new Mat();
    String destination;


    @Override
    public Mat processFrame(Mat input) {
        // call the detectAndDecode method in QRCodeDetector
        String decodedValue = qrcode.detectAndDecodeCurved(input,points);
        // if the points are not empty it sets destination to the decoded values
        // and draws a box around the detected QR code
        if (!points.empty() && !decodedValue.isEmpty()) {
            destination = decodedValue;
            telemetry.addLine("Decoded data: " + decodedValue);
            for (int i = 0; i < points.cols(); i++) {
                Point pt1 = new Point(points.get(0, i));
                Point pt2 = new Point(points.get(0, (i + 1) % 4));
                Imgproc.line(input, pt1, pt2, new Scalar(255, 0, 0), 3);
            }
        }

        return input;

    }

    public String  getDest(){return destination;}


}

