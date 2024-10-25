//package org.firstinspires.ftc.teamcode.auton;
//import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;
//
//import android.util.Size;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.auton.pipeline.SpikePosDetector;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//@Config
//public class Auton {
//    public SpikePosDetector.SPIKE_POS spikePos;
//    private SpikePosDetector pipeline;
//    private VisionPortal vision;
////    private AprilTagProcessor aprilTag;
//    private Transport transport;
//    private SampleMecanumDrive drive;
////    public final Vector2d[] tagPositions = new Vector2d[] {
////            new Vector2d(62, 41.5),
////            new Vector2d(62, 35.5),
////            new Vector2d(62, 29.5),
////            new Vector2d(62,-29.5),
////            new Vector2d(62,-35.5),
////            new Vector2d(62, -41.5),
////    };
////    //TODO: TUNE DELTAF (A VECTOR REPRESENTING THE DISTANCE FROM THE CENTER OF BOT THE CAMERA IS
////    public final Vector2d deltaF = new Vector2d(7.5,4.5);
//    public Pose2d blueLeftBoard, blueCenterBoard, blueRightBoard, redLeftBoard, redCenterBoard, redRightBoard, blueLeftClose, blueCenterClose, blueRightClose, redLeftClose, redCenterClose, redRightClose, blueLeftStack, redRightStack, blueParkLeft, blueParkRight, redParkLeft, redParkRight, blueLeftStackPrep, redRightStackPrep;
//    public boolean isLeftClaw;
//
//    public Auton(HardwareMap hardwareMap) {
//        //TODO: TUNE POSITIONS
//        //Board Positions:
//        //Syntax: Alliance, Position, Board
//        blueLeftBoard = new Pose2d(30, 40, Math.toRadians(176.5));
//        blueCenterBoard = new Pose2d(30, 34, Math.toRadians(176.5));
//        blueRightBoard = new Pose2d(30, 28, Math.toRadians(176.5));
//        redLeftBoard = new Pose2d(30, -29.5, Math.toRadians(183.25));
//        redCenterBoard = new Pose2d(30, -35.5, Math.toRadians(183.25));
//        redRightBoard = new Pose2d(30, -42, Math.toRadians(183.25));
//        //Spike Mark Positions:
//        //Syntax: Alliance, Position, CloseSide (Close to Backdrop) /FarSide (Far from Backdrop)
//        blueLeftClose = new Pose2d(10, 34.5, Math.toRadians(176.5));
//        blueCenterClose = new Pose2d(10, 38, Math.toRadians(84.5));
//        blueRightClose = new Pose2d(20, 34.5, Math.toRadians(176.5));
//        redLeftClose = new Pose2d(20, -34.5, Math.toRadians(183.25));
//        redCenterClose = new Pose2d(10, -38, Math.toRadians(274.5));
//        redRightClose = new Pose2d(10, -34.5, Math.toRadians(183.25));
//        //Park Positions:
//        //Syntax: Alliance, Park, Left/Right
//        blueParkLeft = new Pose2d(50, 63.25, Math.toRadians(180));
//        blueParkRight = new Pose2d(50, 8, Math.toRadians(180));
//        redParkLeft = new Pose2d(50, -8, Math.toRadians(180));
//        redParkRight = new Pose2d(50, -63.25, Math.toRadians(180));
//        //Stack Positions *NOT NECESSARY (If Time)*:
//        blueLeftStackPrep = new Pose2d(10, 36, Math.toRadians(176.5));
//        blueLeftStack = new Pose2d(-14, 28, Math.toRadians(176.5));
//        redRightStackPrep = new Pose2d(10, -36, Math.toRadians(183.25));
//        redRightStack = new Pose2d(-14, -35, Math.toRadians(183.25));
//
////        aprilTag = new AprilTagProcessor.Builder()
////                .build();
//        pipeline = new SpikePosDetector();
//        spikePos = pipeline.getType();
//        vision = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
//                .addProcessors(pipeline)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                .build();
//        drive = new SampleMecanumDrive(hardwareMap);
//        transport = new Transport(hardwareMap);
//    }
//
////    public AprilTagDetection assignID (){
////        int idNum=0;
////
////        if (!isRed){
////            switch(spikePos) {
////                case LEFT:
////                    idNum = 1;
////                case CENTER:
////                    idNum = 2;
////                case RIGHT:
////                    idNum = 3;
////            }
////        } else if (isRed){
////            switch(spikePos) {
////                case LEFT:
////                    idNum = 4;
////                case CENTER:
////                    idNum = 5;
////                case RIGHT:
////                    idNum = 6;
////            }
////        }
////        List<AprilTagDetection> currentDetections = findDetections();
////        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections){
////            if (detection.id == idNum){
////                return detection;
////            }
////        }
////        return null;
////    }
//
////    public void telemetryDetection (AprilTagDetection detection, Telemetry telemetry){
////        if (detection==null){
////            return;
////        }
////        if (detection.metadata!= null){
////            telemetry.addData("ID: ", detection.id);
////            telemetry.addData("Range(Distance from board): ", detection.ftcPose.range);
////            telemetry.addData("Yaw: ", detection.ftcPose.yaw);
////            telemetry.addData("Bearing: ", detection.ftcPose.bearing);
////        } else {
////            telemetry.addData("ID: ", detection.id);
////            return;
////        }
////        telemetry.update();
////    }
//
////    public List<AprilTagDetection> findDetections() {
////        return aprilTag.getDetections();
////    }
////    public Pose2d getPoseFromAprilTag() {
////        List<AprilTagDetection> detections = findDetections();
////        if (detections == null || detections.isEmpty()) {
////            RobotLog.i("getPoseFromAprilTag: no detections");
////        }
////        AprilTagDetection OurTag = detections.get(0);
////        for (AprilTagDetection d : detections) {
////            if (OurTag.ftcPose == null) {
////                OurTag = d;
////                continue;
////            }
////            if (d.ftcPose == null) {
////                continue;
////            }
////            if (Math.abs(d.ftcPose.x) < Math.abs(OurTag.ftcPose.x)) {
////                OurTag = d;
////            }
////        }
////        if (OurTag.ftcPose == null) {
////            RobotLog.i("getPoseFromAprilTag: no detections");
////        }
////
////        Vector2d cameraVector = new Vector2d(OurTag.ftcPose.y, -OurTag.ftcPose.x);
////        Vector2d rTag = tagPositions[OurTag.id - 1];
////        Vector2d returnVector = rTag.minus(deltaF);
////        returnVector = returnVector.minus(cameraVector);
////        Pose2d returnPose = new Pose2d(returnVector, Math.toRadians(-OurTag.ftcPose.yaw));
////        RobotLog.i("getPoseFromAprilTag: reference tag = "+OurTag.id);
////        RobotLog.i(String.format("getPoseFromAprilTag: tag data: (%.3f, %.3f) @%.3f",OurTag.ftcPose.x, OurTag.ftcPose.y, OurTag.ftcPose.yaw));
////        RobotLog.i("getPoseFromAprilTag: pose = "+returnPose.toString());
////
////        return returnPose;
////
////    }
//    public void vision() {
//        spikePos = pipeline.getType();
//    }
//    public SampleMecanumDrive drive() {
//        return drive;
//    }
//
//    public Transport transport() {
//        return transport;
//    }
//
//    public Boolean leftClaw() {
//        return isLeftClaw;
//    }
//
//    public TrajectorySequence LeftBoardToLeftStack(Boolean Reverse) {
//        if (!isRed) {
//            TrajectorySequence blueLeftBoardToLeftStack = drive.trajectorySequenceBuilder(blueLeftClose)
//                    .setReversed(Reverse)
//                    .lineToSplineHeading(blueLeftStackPrep)
//                    .lineToSplineHeading(blueLeftStack)
//                    .build();
//            return blueLeftBoardToLeftStack;
//        } else {
//            TrajectorySequence redLeftBoardToLeftStack = drive.trajectorySequenceBuilder(redLeftClose)
//                    .setReversed(Reverse)
//                    .lineToSplineHeading(redRightStackPrep)
//                    .lineToSplineHeading(redRightStack)
//                    .build();
//            return redLeftBoardToLeftStack;
//        }
//    }
//
//    public TrajectorySequence fromLeftStack() {
//        if (!isRed) {
//            TrajectorySequence blue = drive.trajectorySequenceBuilder(blueLeftStack)
//                    .lineToSplineHeading(blueLeftStackPrep)
//                    .lineToSplineHeading(blueLeftClose)
//                    .build();
//            return blue;
//        } else {
//            TrajectorySequence red = drive.trajectorySequenceBuilder(redRightStack)
//                    .lineToSplineHeading(redRightStackPrep)
//                    .lineToSplineHeading(redLeftClose)
//                    .build();
//            return red;
//        }
//    }
//
//    public TrajectorySequence toLeftStack() {
//        if (!isRed) {
//            TrajectorySequence blue = drive.trajectorySequenceBuilder(blueLeftStack)
//                    .lineToSplineHeading(blueLeftStackPrep)
//                    .lineToSplineHeading(blueLeftClose)
//                    .build();
//            return blue;
//        } else {
//            TrajectorySequence red = drive.trajectorySequenceBuilder(redRightStack)
//                    .lineToSplineHeading(redRightStackPrep)
//                    .lineToSplineHeading(redLeftClose)
//                    .build();
//            return red;
//        }
//    }
//    public TrajectorySequence CenterStack() {
//        if (!isRed) {
//            TrajectorySequence blueCenterBoardToLeftStack = drive.trajectorySequenceBuilder(blueCenterClose)
//                    .lineToSplineHeading(blueLeftStackPrep)
//                    .lineToSplineHeading(blueLeftStack)
//                    .build();
//            return blueCenterBoardToLeftStack;
//        } else {
//            TrajectorySequence redCenterBoardToLeftStack = drive.trajectorySequenceBuilder(redCenterClose)
//                    .lineToSplineHeading(redRightStackPrep)
//                    .lineToSplineHeading(redRightStack)
//                    .build();
//            return redCenterBoardToLeftStack;
//        }
//    }
//
//    public TrajectorySequence RightBoardToLeftStack(Boolean Reverse) {
//        if (!isRed) {
//            TrajectorySequence blueRightBoardToLeftStack = drive.trajectorySequenceBuilder(blueRightClose)
//                    .setReversed(Reverse)
//                    .lineToSplineHeading(blueLeftStackPrep)
//                    .lineToSplineHeading(blueLeftStack)
//                    .build();
//           return blueRightBoardToLeftStack;
//        } else {
//            TrajectorySequence redRightBoardToLeftStack = drive.trajectorySequenceBuilder(redRightClose)
//                    .setReversed(Reverse)
//                    .lineToSplineHeading(redRightStackPrep)
//                    .lineToSplineHeading(redRightStack)
//                    .build();
//            return redRightBoardToLeftStack;
//        }
//    }
//
//    public TrajectorySequence LeftBoardToRightStack(Boolean Reverse) {
//        if (!isRed) {
//            TrajectorySequence blueLeftBoardToRightStack = drive.trajectorySequenceBuilder(blueLeftBoard)
//                    .setReversed(Reverse)
//                    //TODO: WRITE TRAJECTORY
//                    .build();
//            return blueLeftBoardToRightStack;
//        } else {
//            TrajectorySequence redLeftBoardToRightStack = drive.trajectorySequenceBuilder(redLeftBoard)
//                    .setReversed(Reverse)
//                    //TODO: WRITE TRAJECTORY
//                    .build();
//            return redLeftBoardToRightStack;
//        }
//    }
//
//    public TrajectorySequence CenterBoardToRightStack(Boolean Reverse) {
//        if (!isRed) {
//            TrajectorySequence blueCenterBoardToRightStack = drive.trajectorySequenceBuilder(blueCenterBoard)
//                    .setReversed(Reverse)
//                    //TODO: WRITE TRAJECTORY
//                    .build();
//            return blueCenterBoardToRightStack;
//        } else {
//            TrajectorySequence redCenterBoardToRightStack = drive.trajectorySequenceBuilder(redCenterBoard)
//                    .setReversed(Reverse)
//                    //TODO: WRITE TRAJECTORY
//                    .build();
//            return redCenterBoardToRightStack;
//        }
//    }
//
//    public TrajectorySequence RightBoardToRightStack(Boolean Reverse) {
//        if (!isRed) {
//            TrajectorySequence blueRightBoardToRightStack = drive.trajectorySequenceBuilder(blueRightBoard)
//                    .setReversed(Reverse)
//                    //TODO: WRITE TRAJECTORY
//                    .build();
//            return blueRightBoardToRightStack;
//        } else {
//            TrajectorySequence redRightBoardToRightStack = drive.trajectorySequenceBuilder(redRightBoard)
//                    .setReversed(Reverse)
//                    //TODO: WRITE TRAJECTORY
//                    .build();
//            return redRightBoardToRightStack;
//        }
//    }
//
//    public TrajectorySequence LeftPark(Boolean goLeft) {
//        if (goLeft && isRed) {
//            TrajectorySequence redLeftBoardParkLeft = drive.trajectorySequenceBuilder(redLeftClose)
////                    .UNSTABLE_addTemporalMarkerOffset(.5, ()->{transport.reset();})
//                    .lineToSplineHeading(redParkLeft)
//                    .build();
//            return redLeftBoardParkLeft;
//        } else if (goLeft && !isRed) {
//            TrajectorySequence blueLeftBoardParkLeft = drive.trajectorySequenceBuilder(blueLeftClose)
////                    .addTemporalMarker(()->{ transport.reset();})
//                    .lineToSplineHeading(blueParkLeft)
//                    .build();
//            return blueLeftBoardParkLeft;
//        } else if (!goLeft && isRed){
//            TrajectorySequence redLeftBoardParkRight = drive.trajectorySequenceBuilder(redLeftClose)
////                    .addTemporalMarker(()->{transport.reset();})
//                    .lineToSplineHeading(redParkRight)
//                    .build();
//            return redLeftBoardParkRight;
//        } else {
//            TrajectorySequence blueLeftBoardParkRight = drive.trajectorySequenceBuilder(blueLeftClose)
////                    .addTemporalMarker(()->{transport.reset();})
//                    .lineToSplineHeading(blueParkRight)
//                    .build();
//            return blueLeftBoardParkRight;
//        }
//    }
//
//    public TrajectorySequence CenterPark(Boolean goLeft) {
//        if (goLeft && isRed) {
//            TrajectorySequence redCenterBoardParkLeft = drive.trajectorySequenceBuilder(redCenterClose)
////                    .addTemporalMarker(()->{transport.reset();})
//                    .lineToSplineHeading(redParkLeft)
//                    .build();
//            return redCenterBoardParkLeft;
//        } else if (goLeft && !isRed) {
//            TrajectorySequence blueCenterBoardParkLeft = drive.trajectorySequenceBuilder(blueCenterClose)
////                    .addTemporalMarker(()->{transport.reset();})
//                    .lineToSplineHeading(blueParkLeft)
//                    .build();
//            return blueCenterBoardParkLeft;
//        } else if (!goLeft && isRed){
//            TrajectorySequence redCenterBoardParkRight = drive.trajectorySequenceBuilder(redCenterClose)
////                    .addTemporalMarker(()->{transport.reset();})
//                    .lineToSplineHeading(redParkRight)
//                    .build();
//            return redCenterBoardParkRight;
//        } else {
//            TrajectorySequence blueCenterBoardParkRight = drive.trajectorySequenceBuilder(blueCenterClose)
////                    .addTemporalMarker(()->{transport.reset();})
//                    .lineToSplineHeading(blueParkRight)
//                    .build();
//            return blueCenterBoardParkRight;
//        }
//    }
//
//    public TrajectorySequence RightPark(Boolean goLeft) {
//        if (goLeft && isRed) {
//            TrajectorySequence redRightBoardParkLeft = drive.trajectorySequenceBuilder(redRightClose)
////                    .addTemporalMarker(()->{transport.reset();})
//                    .lineToSplineHeading(redParkLeft)
//                    .build();
//            return redRightBoardParkLeft;
//        } else if (goLeft && !isRed) {
//            TrajectorySequence blueRightBoardParkLeft = drive.trajectorySequenceBuilder(blueRightClose)
////                    .addTemporalMarker(()->{transport.reset();})
//                    .lineToSplineHeading(blueParkLeft)
//                    .build();
//            return blueRightBoardParkLeft;
//        } else if (!goLeft && isRed){
//            TrajectorySequence redRightBoardParkRight = drive.trajectorySequenceBuilder(redRightClose)
////                    .addTemporalMarker(()->{transport.reset();})
//                    .lineToSplineHeading(redParkRight)
//                    .build();
//            return redRightBoardParkRight;
//        } else {
//            TrajectorySequence blueRightBoardParkRight = drive.trajectorySequenceBuilder(blueRightClose)
////                    .UNSTABLE_addTemporalMarkerOffset(.5, ()->{transport.reset();})
//                    .lineToSplineHeading(blueParkRight)
//                    .build();
//            return blueRightBoardParkRight;
//        }
//    }
//
//
//}