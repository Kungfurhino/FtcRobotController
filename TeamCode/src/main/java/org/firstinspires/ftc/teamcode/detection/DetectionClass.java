package org.firstinspires.ftc.teamcode.detection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class DetectionClass {
        OpenCvWebcam webcam;
        DetectionTest.SkystoneDeterminationPipeline pipeline;
        LinearOpMode linearOpMode;
        HardwareMap hardwareMap;

        public DetectionClass(HardwareMap hardwareMap, LinearOpMode linearOpMode){
            this.linearOpMode = linearOpMode;
            this.hardwareMap = hardwareMap;
        }

        public void init() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new DetectionTest.SkystoneDeterminationPipeline();
            webcam.setPipeline(pipeline);

            // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
            // out when the RC activity is in portrait. We do our actual image processing assuming
            // landscape orientation, though.
            //phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    linearOpMode.telemetry.addData("errorCode occured", "detection");
                }
            });
        }

        public DetectionTest.SkystoneDeterminationPipeline.SkystonePosition Detect(){
            linearOpMode.telemetry.addData("Analysis", pipeline.getAnalysis());
            linearOpMode.telemetry.update();
            return pipeline.getAnalysis();
        }

}
