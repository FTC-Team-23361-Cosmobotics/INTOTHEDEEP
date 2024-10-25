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
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//
//@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
//public class RedClose extends LinearOpMode{
//    Robot robot;
//    public void slides(int slidesDistance) {
//        robot.slidesTarget = slidesDistance;
//        telemetry.addData("SlidesPos: ", robot.slidesMotor.getCurrentPosition());
//        telemetry.addData("SlidesTarget: ", robot.slidesTarget);
//        telemetry.update();
//    }
//
//    public void armMotor(int armDistance) {
//        robot.armTarget = armDistance;
//        telemetry.addData("ArmMotorPos: ", robot.armMotor.getCurrentPosition());
//        telemetry.addData("ArmMotorTarget: ", robot.armTarget);
//        telemetry.update();
//    }
//
//
//    OpenCvInternalCamera phoneCam;
//    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;
//    @Override
//    public void runOpMode(){
//        robot = new Robot(this);
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
//        Pose2d startPose = new Pose2d(10, -60, Math.toRadians(90));
//        drive.setPoseEstimate(startPose);
//
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
//        //Starting Trajectories:
//        TrajectorySequence myLeftSideTraj1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToSplineHeading(new Pose2d(50, -23, Math.toRadians(180)))
//                .build();
//        TrajectorySequence myCenterSideTraj1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToSplineHeading(new Pose2d(50, -29, Math.toRadians(180)))
//                .build();
//        TrajectorySequence myRightSideTraj1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToSplineHeading(new Pose2d(50, -35, Math.toRadians(180)))
//                .build();
//
//
//        //Cycle Trajectories:
//        TrajectorySequence leftToCyclePath = drive.trajectorySequenceBuilder(new Pose2d(40, -30, Math.toRadians(0)))
//                .strafeTo(new Vector2d(40, -29))
//                .splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(0)), Math.toRadians(180))
//                .build();
//        TrajectorySequence centerToCyclePath = drive.trajectorySequenceBuilder(new Pose2d(38, -31, Math.toRadians(-10)))
//                .strafeTo(new Vector2d(40, -29))
//                .splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(0)), Math.toRadians(180))
//                .build();
//        TrajectorySequence rightToCyclePath = drive.trajectorySequenceBuilder(new Pose2d(40, -37, Math.toRadians(-5)))
//                .strafeTo(new Vector2d(40, -29))
//                .splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(0)), Math.toRadians(180))
//                .build();
//
//        //Park Trajectories:
//        TrajectorySequence parkLefttoRight = drive.trajectorySequenceBuilder(new Pose2d(52, -23, Math.toRadians(180)))
//                .strafeTo(new Vector2d(48, -47))
//                .splineToConstantHeading(new Vector2d(70, -56), Math.toRadians(180))
//                .build();
//        TrajectorySequence parkCentertoRight = drive.trajectorySequenceBuilder(new Pose2d(52, -29, Math.toRadians(180)))
//                .strafeTo(new Vector2d(48, -47))
//                .splineToConstantHeading(new Vector2d(70, -56), Math.toRadians(180))
//                .build();
//        TrajectorySequence parkRighttoRight = drive.trajectorySequenceBuilder(new Pose2d(52, -35, Math.toRadians(180)))
//                .strafeTo(new Vector2d(48, -47))
//                .splineToConstantHeading(new Vector2d(70, -56), Math.toRadians(180))
//                .build();
//
//
//        TrajectorySequence parkLefttoLeft = drive.trajectorySequenceBuilder(new Pose2d(40, -30, Math.toRadians(0)))
//                .strafeTo(new Vector2d(48, -14))
//                .splineToConstantHeading(new Vector2d(60, -11), Math.toRadians(0))
//                .build();
//        TrajectorySequence parkCentertoLeft = drive.trajectorySequenceBuilder(new Pose2d(38, -31, Math.toRadians(-10)))
//                .strafeTo(new Vector2d(48, -14))
//                .splineToConstantHeading(new Vector2d(60, -11), Math.toRadians(0))
//                .build();
//        TrajectorySequence parkRighttoLeft = drive.trajectorySequenceBuilder(new Pose2d(40, -37, Math.toRadians(-5)))
//                .strafeTo(new Vector2d(48, -14))
//                .splineToConstantHeading(new Vector2d(60, -11), Math.toRadians(0))
//                .build();
//
//
//        while (opModeInInit()) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.update();
//        }
//
//        if (opModeIsActive()) {
//            SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition colorPos = pipeline.getAnalysis();
//            if (colorPos == SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT){
//                drive.followTrajectorySequence(myLeftSideTraj1);
//                //Extend Slides
//                slides(3000);
//                long start_t = System.currentTimeMillis();
//                while (System.currentTimeMillis()-start_t < 2000 && opModeIsActive()) {
//
//                }
//                //Place Yellow Pixel
//                robot.leftIntake.setPosition(1);
//                start_t = System.currentTimeMillis();
//                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
//
//                }
//                slides(0);
//                robot.leftIntake.setPosition(0);
////                rotationalIntake.setPosition(.92);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 2000 && opModeIsActive()) {
////
////                }
////                armMotor(1700, .5);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 2000 && opModeIsActive()) {
////
////                }
////                slides(2700, 1);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 2000 && opModeIsActive()) {
////
////                }
////                //Place Purple Pixel
////                leftIntake.setPosition(1);
////                slides(0, 1);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 2000 && opModeIsActive()) {
////
////                }
////                armMotor(-1700, .5);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 2000 && opModeIsActive()) {
////
////                }
////                leftIntake.setPosition(0);
////                rotationalIntake.setPosition(.31);
//                //Park
//                drive.followTrajectorySequence(parkLefttoRight);
//            } else if (colorPos == SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.CENTER) {
////                rotationalIntake.setPosition(.92);
////                long start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 200 && opModeIsActive()) {
////
////                }
////                armMotor(1500, 1);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
////
////                }
////                slides(3200,1);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
////
////                }
////                //Place Purp Pixel
////                leftIntake.setPosition(1);
////                slides(0, 1);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
////
////                }
////                leftIntake.setPosition(0);
////                armMotor(0, .3);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
////
////                }
//                robot.intakeRotation.setPosition(.31);
//                //drive.followTrajectorySequence(myCenterSideTraj1);
//                //Extend Slides
//                slides(3000);
//                long start_t = System.currentTimeMillis();
//                while (System.currentTimeMillis()-start_t < 2000 && opModeIsActive()) {
//
//                }
//                //Place Yellow Pixel
//                robot.leftIntake.setPosition(1);
//                start_t = System.currentTimeMillis();
//                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
//
//                }
//                slides(0);
//                robot.leftIntake.setPosition(0);
//                //Park
//                drive.followTrajectorySequence(parkCentertoRight);
//            } else if (colorPos == SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.RIGHT) {
//                drive.followTrajectorySequence(myRightSideTraj1);
//                //Extend Slides
//                slides(3000);
//                long start_t = System.currentTimeMillis();
//                while (System.currentTimeMillis()-start_t < 2000 && opModeIsActive()) {
//
//                }
//                //Place Yellow Pixel
//                robot.leftIntake.setPosition(1);
//                start_t = System.currentTimeMillis();
//                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
//
//                }
//                slides(0);
//                robot.leftIntake.setPosition(0);
////                rotationalIntake.setPosition(.92);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
////
////                }
////                armMotor(1500, 1);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
////
////                }
////                slides(600, 1);
////                rightIntake.setPosition(1);
////                //Place Purple Pixel
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
////
////                }
////                slides(0, 1);
////                armMotor(0, .3);
////                start_t = System.currentTimeMillis();
////                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {
////
////                }
////                rightIntake.setPosition(0);
////                rotationalIntake.setPosition(.31);
//                //Park
//                drive.followTrajectorySequence(parkRighttoRight);
//            }
//        }
//    }
//}
