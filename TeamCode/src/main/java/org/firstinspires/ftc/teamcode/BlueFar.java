//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.teleop.Robot;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
//public class BlueFar extends LinearOpMode{
//    OpenCvInternalCamera phoneCam;
//    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;
//    @Override
//    public void runOpMode(){
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        pipeline = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
//        phoneCam.setPipeline(pipeline);
//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                phoneCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//
//        while (!opModeIsActive()) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.update();
//        }
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Pose2d startPose = new Pose2d(-33, 58, Math.toRadians(270));
//
//        //Purple Trajectories:
//        TrajectorySequence leftPurpTraj = drive.trajectorySequenceBuilder(startPose)
//                //Make The Trajectory
//                .build();
//        TrajectorySequence centerPurpTraj = drive.trajectorySequenceBuilder(startPose)
//                //Make The Trajectory
//                .build();
//        TrajectorySequence rightPurpTraj = drive.trajectorySequenceBuilder(startPose)
//                //Make The Trajectory
//                .build();
//
//        TrajectorySequence myLeftSideTraj1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToSplineHeading(new Pose2d(-34, 58, Math.toRadians(270)))
//                .strafeTo(new Vector2d(17, 58))
//                .splineToSplineHeading(new Pose2d(12, 37, Math.toRadians(5)), Math.toRadians(270))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //Extend Slides
//                    slides(600);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    robot.intakeRotation.setPosition(.92);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
//                    //Place Purp Pixel
//                    robot.leftIntake.setPosition(0);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.intakeRotation.setPosition(.31);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
//                    robot.leftIntake.setPosition(1);
//                    armMotor(2000);
//                    slides(1000);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
//                    robot.rightIntake.setPosition(0);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
//                    armMotor(0);
//                    slides(0);
//                    robot.rightIntake.setPosition(1);
//                })
//                .build();
//        TrajectorySequence myRightSideTraj1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToSplineHeading(new Pose2d(-34, 58, Math.toRadians(270)))
//                .strafeTo(new Vector2d(17, 56.5))
//                .splineToSplineHeading(new Pose2d(12, 34, Math.toRadians(0)), Math.toRadians(270))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //Extend Slides
//                    slides(600);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    robot.intakeRotation.setPosition(.92);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
//                    //Place Purp Pixel
//                    robot.leftIntake.setPosition(0);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.intakeRotation.setPosition(.31);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
//                    robot.leftIntake.setPosition(1);
//                    armMotor(2000);
//                    slides(1000);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
//                    robot.rightIntake.setPosition(0);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
//                    armMotor(0);
//                    slides(0);
//                    robot.rightIntake.setPosition(1);
//                })
//                .build();
//        TrajectorySequence myCenterSideTraj1 = drive.trajectorySequenceBuilder(startPose)
//                .strafeTo(new Vector2d(18, 58))
//                .splineToSplineHeading(new Pose2d(12, 32, Math.toRadians(2)), Math.toRadians(270))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //Extend Slides
//                    slides(600);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    robot.intakeRotation.setPosition(.92);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
//                    //Place Purp Pixel
//                    robot.leftIntake.setPosition(0);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.intakeRotation.setPosition(.31);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
//                    robot.leftIntake.setPosition(1);
//                    armMotor(2000);
//                    slides(1000);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
//                    robot.rightIntake.setPosition(0);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
//                    armMotor(0);
//                    slides(0);
//                    robot.rightIntake.setPosition(1);
//                })
//                .build();
//
//        TrajectorySequence centerToCyclePath = drive.trajectorySequenceBuilder(new Pose2d(40, 35, Math.toRadians(5)))
//                .strafeTo(new Vector2d(40, 24))
//                .splineToSplineHeading(new Pose2d(-35, 12, Math.toRadians(0)), Math.toRadians(180))
//                .build();
//        TrajectorySequence leftToCyclePath = drive.trajectorySequenceBuilder(new Pose2d(40, 37, Math.toRadians(5)))
//                .strafeTo(new Vector2d(40, 24))
//                .splineToSplineHeading(new Pose2d(-35, 12, Math.toRadians(0)), Math.toRadians(180))
//                .build();
//        TrajectorySequence rightToCyclePath = drive.trajectorySequenceBuilder(new Pose2d(40, 30, Math.toRadians(0)))
//                .strafeTo(new Vector2d(40, 24))
//                .splineToSplineHeading(new Pose2d(-35, 12, Math.toRadians(0)), Math.toRadians(180))
//                .build();
//
//        TrajectorySequence parkCentertoLeft = drive.trajectorySequenceBuilder(new Pose2d(40, 35, Math.toRadians(5)))
//                .lineToSplineHeading(new Pose2d(50, 57, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
//                .build();
//        TrajectorySequence parkLefttoLeft = drive.trajectorySequenceBuilder(new Pose2d(40, 37, Math.toRadians(5)))
//                .lineToSplineHeading(new Pose2d(50, 57, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
//                .build();
//        TrajectorySequence parkRighttoLeft = drive.trajectorySequenceBuilder(new Pose2d(40, 30, Math.toRadians(0)))
//                .lineToSplineHeading(new Pose2d(50, 57, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
//                .build();
//        TrajectorySequence parkCentertoRight = drive.trajectorySequenceBuilder(new Pose2d(40, 35, Math.toRadians(5)))
//                .lineToSplineHeading(new Pose2d(48, 14, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(60, 11), Math.toRadians(0))
//                .build();
//        TrajectorySequence parkLefttoRight = drive.trajectorySequenceBuilder(new Pose2d(40, 37, Math.toRadians(5)))
//                .lineToSplineHeading(new Pose2d(48, 14, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(60, 11), Math.toRadians(0))
//                .build();
//        TrajectorySequence parkRighttoRight = drive.trajectorySequenceBuilder(new Pose2d(40, 30, Math.toRadians(0)))
//                .lineToSplineHeading(new Pose2d(48, 14, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(60, 11), Math.toRadians(0))
//                .build();
//
//        waitForStart();
//        if (isStopRequested()) return;
//        while (opModeIsActive()) {
//            SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition colorPos = pipeline.getAnalysis();
//            if (colorPos == SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT){
//                drive.followTrajectorySequence(myLeftSideTraj1);
//                drive.followTrajectorySequence(parkLefttoLeft);
//            } else if (colorPos == SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.RIGHT) {
//                drive.followTrajectorySequence(myRightSideTraj1);
//                drive.followTrajectorySequence(parkRighttoLeft);
//            } else if (colorPos == SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.CENTER) {
//                drive.followTrajectorySequence(myCenterSideTraj1);
//                drive.followTrajectorySequence(parkCentertoLeft);
//            }
//        }
//    }
//}
