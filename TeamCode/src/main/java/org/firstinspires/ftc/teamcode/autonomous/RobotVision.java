package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class RobotVision
{
    private OpenCvCamera camera;
    private HardwareMap hardwareMap;
    private Pipeline pipeline;
    public RobotVision(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;
    }
    public void initializeCamera()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // With live preview
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // Without live preview
        //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE");
    }
    public int identify()
    {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        camera.openCameraDevice();

        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        // NOTE: this must be called *before* you call startStreaming(...)
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        camera.setPipeline(pipeline);

        return 0;
    }
}
