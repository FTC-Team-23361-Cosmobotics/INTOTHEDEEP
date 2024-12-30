//package org.firstinspires.ftc.teamcode.auton.variants;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.sun.tools.javac.Main;
//
//import org.firstinspires.ftc.teamcode.auton.Auton;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.teleop.AllianceStorage;
//
//@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
//@Config
//public class RedClose extends LinearOpMode {
//    Auton Auton1;
//    Transport transport;
//    SampleMecanumDrive drive;
//    public Pose2d StartPose, bucketPose, firstSamplePose, secondSamplePose, thirdSamplePose, specimenPose;
//    //    public static double StartPoseX = -22;
////
////    public static double StartPoseY = -60;
////    public static double StartPoseHeading = Math.toRadians(270);
////    public static double SpecimenPoseX = -3;
////    public static double SpecimenPoseY = -28;
////    public static double SpecimenHeading = Math.toRadians(270);
//    public static double BucketX = -53;
//    public static double BucketY = -56;
//    public static double BucketHeading = Math.toRadians(45);
//    public static double FirstSampleX = -44;
//    public static double FirstSampleY = -35;
//    public static double FirstSampleHeading = 1.57;
//    public static double SecondSampleX = -55;
//    public static double SecondSampleY = -35;
//    public static double SecondSampleHeading = 1.57;
//    //
////    public static double ThirdSampleX = -45;
////    public static double ThirdSampleY = -40;
////    public static double ThirdSampleHeading = 2.3;
//    public static double ThirdSampleX = -50;
//    public static double ThirdSampleY = 0;
//    public static double ThirdSampleHeading = 3.14;
//
////    public static double sampleTangent = Math.toRadians(135);
//
//    public static int HighSpecimen = 1500;
//    public static int SpecimenDisp = 1;
//    public static int ScoreSpecimenDisp = 1;
//    public static int Ex = 700;
//    public static double intakeWait = .25;
//    public static double retractWait = .75;
//    public static double transferWait = .1;
//    public static double bucketUpWait = .9;
//    public static double scoreWait = .75;
//    public static double scoreWait2 = 3;
//
//    public static double bucketOffset = -.5;
//    public static double bOneOneOffset = -3;
//    public static double bOneTwoOffset = -.5;
//    public static double bOneThreeOffset = 0;
//    public static double bTwoOneOffset = -1.5;
//    public static double bThreeZeroOffset = -1.5;
//    public static double bThreePointFiveOffset = -1.25;
//    public static double bThreeOneOffset = -1;
//    public static double bThreeTwoOffset = -.5;
//    public static double bThreeThreeOffset = 1.5;
//    public static double bThreeFourOffset = 2;
//    public static double driveOffset = 20;
//    public static double bSigmaOffset = -2;
//    public static double bAlphaOffset = 1;
//
//    public void followTrajectory(TrajectorySequence traj) {
//        drive.followTrajectorySequenceAsync(traj);
//        while (!isStopRequested() && drive.isBusy()) {
//            drive.update();
//            transport.update();
//        }
//    }
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d StartPose = new Pose2d(-22, -60, Math.toRadians(270));
//        Pose2d specimenPose = new Pose2d(-3, -28, Math.toRadians(270));
//        Pose2d bucketPose = new Pose2d(BucketX, BucketY, BucketHeading);
//        Pose2d firstSamplePose = new Pose2d(FirstSampleX, FirstSampleY, FirstSampleHeading);
//        Pose2d secondSamplePose = new Pose2d(SecondSampleX, SecondSampleY, SecondSampleHeading);
//        Pose2d thirdSamplePose = new Pose2d(ThirdSampleX, ThirdSampleY, ThirdSampleHeading);
//        Auton1 = new Auton(hardwareMap);
//        transport = Auton1.transport();
//        drive = Auton1.drive();
////        AllianceStorage.isRed = false;
////        BlueClose.isLeftClaw = true; //YELLOW PIXEL IN THIS CLAW
//        drive.setPoseEstimate(StartPose);
//
//        //Board Auton:
//        TrajectorySequence specimen = drive.trajectorySequenceBuilder(StartPose)
//                .addTemporalMarker(() -> {
//                    transport.setRot(1);
//                    transport.setClawPos(1);
//                    transport.intake(0);
//                    transport.setExtendoTarget(0);
//                })
//                .lineToSplineHeading(bucketPose)
//                .UNSTABLE_addTemporalMarkerOffset(bOneOneOffset,() -> {
////                    transport.setLeftClaw(.4);
////                    transport.setRightClaw(.05);
//                    transport.setOutTarget(2500);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bOneTwoOffset, () -> {
//                    transport.setOutArm(.7);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bOneThreeOffset,() -> {
//                    transport.setClawPos(0);
//                })
//                .waitSeconds(scoreWait)
//                .addTemporalMarker(() -> {
//                    transport.resetOut();
//                    transport.setRot(0);
//                    transport.intake(1);
//                })
//                .build();
//        TrajectorySequence firstSample = drive.trajectorySequenceBuilder(bucketPose)
////                .lineToSplineHeading(new Pose2d(-22, -40, Math.toRadians(270)))
//                .lineToSplineHeading(firstSamplePose)
//                .lineToSplineHeading(new Pose2d(FirstSampleX, FirstSampleY + driveOffset, FirstSampleHeading))
////                .waitSeconds(intakeWait)
////                .addTemporalMarker(() -> {
////                    transport.setRot(.925);
////                    transport.setExtendoTarget(0);
////                })
////                .waitSeconds(retractWait)
//                .build();
//        TrajectorySequence firstbucket = drive.trajectorySequenceBuilder(new Pose2d(FirstSampleX, FirstSampleY + driveOffset, FirstSampleHeading))
//                .addTemporalMarker(() -> {
//                    transport.setRot(1);
//                    transport.intake(0);
//                    transport.setExtendoTarget(200);
//                })
//                .lineToSplineHeading(bucketPose)
//                .UNSTABLE_addTemporalMarkerOffset(bThreeZeroOffset, () -> {
//                    transport.setOutArm(.15);
//                    transport.setExtendoTarget(-200);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreePointFiveOffset, () -> {
//                    transport.setOutArm(0.05);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreeOneOffset, () -> {
//                    transport.setClawPos(1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreeTwoOffset,() -> {
////                    transport.setLeftClaw(.4);
////                    transport.setRightClaw(.05);
//                    transport.setOutTarget(2500);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreeThreeOffset, () -> {
//                    transport.setOutArm(.7);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreeFourOffset,() -> {
//                    transport.setClawPos(0);
//                })
//                .waitSeconds(scoreWait2)
//                .addTemporalMarker(() -> {
//                    transport.resetOut();
//                    transport.setRot(0);
//                    transport.intake(1);
//                })
//                .build();
//        TrajectorySequence secondSample = drive.trajectorySequenceBuilder(bucketPose)
////                .lineToSplineHeading(new Pose2d(-22, -40, Math.toRadians(270)))
//                .lineToSplineHeading(secondSamplePose)
//                .lineToSplineHeading(new Pose2d(SecondSampleX, SecondSampleY + driveOffset, SecondSampleHeading))
////                .waitSeconds(intakeWait)
////                .addTemporalMarker(() -> {
////                    transport.setRot(.925);
////                    transport.setExtendoTarget(0);
////                })
////                .waitSeconds(retractWait)
//                .build();
//        TrajectorySequence secondBucket = drive.trajectorySequenceBuilder(new Pose2d(SecondSampleX, SecondSampleY + driveOffset, SecondSampleHeading))
//                .addTemporalMarker(() -> {
//                    transport.setRot(1);
//                    transport.intake(0);
//                    transport.setExtendoTarget(200);
//                })
//                .lineToSplineHeading(bucketPose)
//                .UNSTABLE_addTemporalMarkerOffset(bThreeZeroOffset, () -> {
//                    transport.setOutArm(.15);
//                    transport.setExtendoTarget(-200);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreePointFiveOffset, () -> {
//                    transport.setOutArm(0.05);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreeOneOffset, () -> {
//                    transport.setClawPos(1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreeTwoOffset,() -> {
////                    transport.setLeftClaw(.4);
////                    transport.setRightClaw(.05);
//                    transport.setOutTarget(2500);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreeThreeOffset, () -> {
//                    transport.setOutArm(.7);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(bThreeFourOffset,() -> {
//                    transport.setClawPos(0);
//                })
//                .waitSeconds(scoreWait2)
//                .addTemporalMarker(() -> {
//                    transport.resetOut();
//                    transport.setRot(0);
//                    transport.intake(1);
//                })
//                .build();
//        TrajectorySequence thirdSample = drive.trajectorySequenceBuilder(bucketPose)
////                .lineToSplineHeading(new Pose2d(-22, -40, Math.toRadians(270)))
//                .lineToSplineHeading(thirdSamplePose)
//                .UNSTABLE_addDisplacementMarkerOffset(bAlphaOffset,() -> {
//                    transport.setOutArm(.6);
//                })
//                .lineToSplineHeading(new Pose2d(ThirdSampleX + 40, ThirdSampleY, ThirdSampleHeading))
////                .waitSeconds(intakeWait)
////                .addTemporalMarker(() -> {
////                    transport.setRot(.925);
////                    transport.setExtendoTarget(0);
////                })
////                .waitSeconds(retractWait)
//                .build();
//
//        TrajectorySequence thirdBucket = drive.trajectorySequenceBuilder(thirdSamplePose)
//                .lineToSplineHeading(bucketPose)
//                .UNSTABLE_addTemporalMarkerOffset(bucketOffset, () -> {
////                    transport.setIntakePower(.7);
//                })
//                .waitSeconds(transferWait)
//                .addTemporalMarker(() -> {
//                    transport.setOutTarget(3000);
//                })
//                .waitSeconds(bucketUpWait)
//                .addTemporalMarker(() -> {
//                    transport.setOutArm(.6);
//                })
//                .waitSeconds(scoreWait)
//                .addTemporalMarker(() -> {
//                    transport.setOutArm(.15);
//                    transport.setOutTarget(0);
//                })
////                .waitSeconds(bucketDownWait)
//                .build();
//
//
//
////
////        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(BlueClose.blueLeftBoard)
////                .waitSeconds(2)
////                .addTemporalMarker(()-> {transport.closeIntaking();})
////                .lineToSplineHeading(BlueClose.blueLeftClose)
////                .addTemporalMarker(()->{
////                   transport.fullRightClaw();
////                })
////                .build();
////
////        TrajectorySequence centerBoard = drive.trajectorySequenceBuilder(startPose)
//////                .addTemporalMarker(()->{transport.closeIntaking();})
//////                .waitSeconds(4)
//////                .addTemporalMarker(()->{
//////                    transport.fullRightClaw();
//////                })
//////                .waitSeconds(.2)
//////                .addTemporalMarker(()->{transport.two();})
////                .lineToSplineHeading(BlueClose.blueCenterBoard)
//////                .addTemporalMarker(()->{
//////                        transport.fullLeftClaw();
//////                })
//////                .addTemporalMarker(()->{transport.reset();})
////                .build();
////
////        TrajectorySequence centerPixel = drive.trajectorySequenceBuilder(BlueClose.blueCenterBoard)
////                .waitSeconds(2)
////                //                .addTemporalMarker(()-> {transport.closeIntaking();})
////                .lineToSplineHeading(BlueClose.blueCenterClose)
//////                .addTemporalMarker(()->{
//////                   transport.fullRightClaw();
//////                })
////                .build();
////
////        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(startPose)
//////                .UNSTABLE_addTemporalMarkerOffset(1, () -> {transport.two();})
////                .lineToSplineHeading(BlueClose.blueRightBoard)
//////                .addTemporalMarker(()->{
//////                    transport.fullLeftClaw();
//////                })
//////                .waitSeconds(.2)
//////                .addTemporalMarker(()->{transport.farIntaking();})
//////                .waitSeconds(4)
//////                .addTemporalMarker(()->{
//////                    transport.fullRightClaw();
//////                })
//////                .waitSeconds(.2)
//////                .addTemporalMarker(()->{
//////                    transport.closeIntaking();
//////                })
////                .build();
////
////        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(BlueClose.blueRightBoard)
////                .waitSeconds(2)
////                //                .addTemporalMarker(()-> {transport.closeIntaking();})
////                .lineToSplineHeading(BlueClose.blueRightClose)
//////                .addTemporalMarker(()->{
//////                   transport.fullRightClaw();
//////                })
////                .build();
//
//        while (opModeInInit() && !isStopRequested()) {
////            BlueClose.vision();
////            telemetry.addData("Spike Pos", BlueClose.spikePos);
////            telemetry.update();
//        }
//
//        waitForStart();
//        if (isStopRequested()) return;
//        if (opModeIsActive() && !isStopRequested()) {
//            followTrajectory(specimen);
//            followTrajectory(firstSample);
//            followTrajectory(firstbucket);
//            followTrajectory(secondSample);
//            followTrajectory(secondBucket);
//            followTrajectory(thirdSample);
////            followTrajectory(thirdBucket);
////            switch (BlueClose.spikePos) {
////                case LEFT:
////                    followTrajectory(leftBoard);
////                    followTrajectory(BlueClose.LeftBoardPark(true));
////                    sleep(30000);
////                case CENTER:
////                    followTrajectory(centerBoard);
////                    followTrajectory(BlueClose.LeftBoardPark(true));
////                    sleep(30000);
////                case RIGHT:
////                    followTrajectory(rightBoard);
////                    followTrajectory(BlueClose.LeftBoardPark(true));
////                    sleep(30000);
////            }
//
//            //TODO: Test All Trajectories:
//            //Travel to Board First
////            followTrajectory(leftBoard);
////            followTrajectory(centerBoard);
////            followTrajectory(firstSample);
////            followTrajectory(bucket1);
////            followTrajectory(mainSample);
////            followTrajectory(bucket2);
////            followTrajectory(mainSample);
////            followTrajectory(bucket2);
//            //Traveling to Spike Mark Second
////            followTrajectory(leftPixel);
////            followTrajectory(centerPixel);
////            followTrajectory(rightPixel);
//            //Parking Last
//            //Parking from Left Spike
////            followTrajectory(BlueClose.LeftPark(true));
////            followTrajectory(BlueClose.LeftPark(false));
//            //Parking from Center Spike
////            followTrajectory(BlueClose.CenterPark(true));
////            followTrajectory(BlueClose.CenterPark(false));
//            //Parking from RightSpike
////            followTrajectory(BlueClose.RightPark(true));
////            followTrajectory(BlueClose.RightBoardToLeftStack(false));
////            followTrajectory(BlueClose.RightBoardToLeftStack(true));
////            followTrajectory(BlueClose.RightPark(false));
//        }
//    }
//}
