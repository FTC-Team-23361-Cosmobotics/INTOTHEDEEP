package org.firstinspires.ftc.teamcode.teleop.drive;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class AprilTagTesting extends OpMode {
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    AprilTagDetection tag1;
    @Override
    public void init() {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    @Override
    public void loop() {
        if (tagProcessor.getDetections().size() > 0) {
            tag1 = tagProcessor.getDetections().get(0);

//            telemetry.addData("x", tag1.ftcPose.x);
//            telemetry.addData("y", tag1.ftcPose.y);
//            telemetry.addData("z", tag1.ftcPose.z);
//            telemetry.addData("roll", tag1.ftcPose.roll);
//            telemetry.addData("pitch", tag1.ftcPose.pitch);
//            telemetry.addData("yaw", tag1.ftcPose.yaw);
//            telemetry.addData("range", tag1.ftcPose.range);
//            telemetry.addData("bearing", tag1.ftcPose.bearing);
        }
        telemetry.update();
    }
}
