package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.RISEDogeCV;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.RISEDogeCV.RISEMath.RISEVector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Create onSean Cardosi on 2019-11-14
 */
public class RISEDogeDetector {
    private OpenCvCamera phoneCamera;
    private OpenCvWebcam webcam;
    SkystoneDetector skystoneDetector;
    private Cam cam;

    public enum Cam{
        PHONE, WEBCAM
    }

    RISEDogeDetector(Cam cam, HardwareMap hardwareMap){
        this.cam = cam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if(cam.equals(Cam.PHONE)){
            phoneCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            skystoneDetector = new SkystoneDetector();
            phoneCamera.setPipeline(skystoneDetector);
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            skystoneDetector = new SkystoneDetector();
            webcam.setPipeline(skystoneDetector);
        }
    }

    public void start(){
        if(cam.equals(Cam.PHONE)){
            phoneCamera.openCameraDevice();
            phoneCamera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam.openCameraDevice();
            webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);//This is wrong for a webcam I think
        }
    }

    public void stop(){
        if(cam.equals(Cam.PHONE)){
            phoneCamera.stopStreaming();
            phoneCamera.closeCameraDevice();
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    public void pauseViewPort(){
        if(cam.equals(Cam.PHONE)) phoneCamera.pauseViewport();
        else if(cam.equals(Cam.WEBCAM)) webcam.pauseViewport();
    }

    public void resumeViewPort(){
        if(cam.equals(Cam.PHONE)) phoneCamera.resumeViewport();
        else if(cam.equals(Cam.WEBCAM)) webcam.resumeViewport();
    }

    SkystonePosition getStoneSkystonePosition(){
        double x = getStoneX();
        if(x < 160) return SkystonePosition.LEFT;//Maybe x < 100
        else if(x >= 160 && x <= 320) return SkystonePosition.MIDDLE;//x >= 100 and x<=150
        else if(x > 280) return SkystonePosition.RIGHT;//x >150
        return null;
    }

    public RISEVector getStoneSkystoneVector(){
        SkystonePosition position = getStoneSkystonePosition();
        if(position == SkystonePosition.LEFT) return new RISEVector(-8,29);
        else if(position == SkystonePosition.MIDDLE) return new RISEVector(0,29);//Maybe 0, 20
        else if(position == SkystonePosition.RIGHT) return new RISEVector(8,29);
        return null;
    }

    double getStoneX(){
        return skystoneDetector.getScreenPosition().x;
    }

    double getStoneY(){
        return skystoneDetector.getScreenPosition().y;
    }

    boolean isDetected(){
        return skystoneDetector.isDetected();
    }


    public enum SkystonePosition {
        LEFT, MIDDLE, RIGHT
    }
}