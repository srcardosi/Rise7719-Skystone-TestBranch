package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class GGOpenCV implements VisionSystem {

    public static final Rect CAMERA_RECT = new Rect(0, 0, 320, 240);

    public Cam cam;
    public OpenCvCamera camera;
    public GGSkystoneDetector detector;
    HardwareMap hardwareMap;

    public enum Cam{
        PHONE, WEBCAM
    }

    @Override
    public void startLook(TargetType targetType) {
        switch (targetType) {
            case SKYSTONE: {
                detector = new GGSkystoneDetector();
                detector.useDefaults();
            }
            default: {

            }
        }
        startCamera();
    }

    @Override
    public void stopLook() {
        camera.closeCameraDevice();
    }

    @Override
    public boolean found() {
        return detector.isDetected();
    }

    public GGOpenCV(Cam cam, HardwareMap hardwareMap) {
        this.cam = cam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();
    }

    public void startCamera() {
        camera.setPipeline(detector);
        camera.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

}