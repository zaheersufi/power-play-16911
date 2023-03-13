package org.firstinspires.ftc.teamcode.autonomous.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

public class RedStackPipeline extends BlueStackPipeline {
    public RedStackPipeline(Telemetry telemetry, double factor, double offset) {
        super(telemetry, factor, offset);
        COLOR = new Scalar(255,0,0);
        COLOR_HUE = 0;
    }
}
