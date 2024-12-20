package org.firstinspires.ftc.teamcode.auton.variants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.Auton;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
@Config
public class Auton1 extends LinearOpMode {
    Auton Auton1;
    Transport transport;
    SampleMecanumDrive drive;
    public Pose2d StartPose, bucketPose, firstSamplePose, secondSamplePose, thirdSamplePose, specimenPose;
//    public static double StartPoseX = -22;
//
//    public static double StartPoseY = -60;
//    public static double StartPoseHeading = Math.toRadians(270);
//    public static double SpecimenPoseX = -3;
//    public static double SpecimenPoseY = -28;
//    public static double SpecimenHeading = Math.toRadians(270);
    public static double BucketX = -53;
    public static double BucketY = -56;
    public static double BucketHeading = Math.toRadians(45);
    public static double FirstSampleX = -28;
    public static double FirstSampleY = -34;
    public static double FirstSampleHeading = 2.675;
    public static double SecondSampleX = -34;
    public static double SecondSampleY = -34;
    public static double SecondSampleHeading = 2.675;

    public static double ThirdSampleX = -45;
    public static double ThirdSampleY = -34;
    public static double ThirdSampleHeading = 2.675;

//    public static double sampleTangent = Math.toRadians(135);

    public static int HighSpecimen = 1500;
    public static int SpecimenDisp = 1;
    public static int ScoreSpecimenDisp = 37;
    public static int Ex = 1750;
    public static double intakeWait = .75;
    public static double retractWait = .75;
    public static double transferWait = .1;
    public static double bucketUpWait = .9;
    public static double scoreWait = .5;
    public static double bucketDownWait = .9;

    public static double bucketOffset = -.5;

    public void followTrajectory(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
            transport.update();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d StartPose = new Pose2d(-22, -60, Math.toRadians(270));
        Pose2d specimenPose = new Pose2d(-3, -28, Math.toRadians(270));
        Pose2d bucketPose = new Pose2d(BucketX, BucketY, BucketHeading);
        Pose2d firstSamplePose = new Pose2d(FirstSampleX, FirstSampleY, FirstSampleHeading);
        Pose2d secondSamplePose = new Pose2d(SecondSampleX, SecondSampleY, SecondSampleHeading);
        Pose2d thirdSamplePose = new Pose2d(ThirdSampleX, ThirdSampleY, ThirdSampleHeading);
        Auton1 = new Auton(hardwareMap);
        transport = Auton1.transport();
        drive = Auton1.drive();
//        AllianceStorage.isRed = false;
//        BlueClose.isLeftClaw = true; //YELLOW PIXEL IN THIS CLAW
        drive.setPoseEstimate(StartPose);

        //Board Auton:
        TrajectorySequence specimen = drive.trajectorySequenceBuilder(StartPose)
                .lineToSplineHeading(specimenPose)
//                .addDisplacementMarker(() -> {
////                    transport.setLeftClaw(.4);
////                    transport.setRightClaw(.05);
//                    transport.setRot(.2);
//                    transport.setOutTarget(HighSpecimen);
//                })
//                .addDisplacementMarker(ScoreSpecimenDisp, () -> {
//                    transport.setOutArm(.15);
//                    transport.setOutTarget(0);
//                })
                .build();
        TrajectorySequence firstSample = drive.trajectorySequenceBuilder(specimenPose)
                .lineToSplineHeading(new Pose2d(-22, -40, Math.toRadians(270)))
                .lineToSplineHeading(firstSamplePose)
                .addTemporalMarker(() -> {
                    transport.setRot(.2);
                    transport.setExtendoTarget(Ex);
//                    transport.setIntakePower(-1);
                })
                .waitSeconds(intakeWait)
                .addTemporalMarker(() -> {
                    transport.setRot(.925);
                    transport.setExtendoTarget(0);
                })
                .waitSeconds(retractWait)
                .build();
        TrajectorySequence firstbucket = drive.trajectorySequenceBuilder(firstSamplePose)
                .lineToSplineHeading(bucketPose)
                .UNSTABLE_addTemporalMarkerOffset(bucketOffset, () -> {
//                    transport.setIntakePower(.7);
                })
                .waitSeconds(transferWait)
                .addTemporalMarker(() -> {
                    transport.setOutTarget(3000);
                })
                .waitSeconds(bucketUpWait)
                .addTemporalMarker(() -> {
                    transport.setBucketPitch(.6);
                })
                .waitSeconds(scoreWait)
                .addTemporalMarker(() -> {
                    transport.setBucketPitch(.15);
                    transport.setOutTarget(0);
                })
                .waitSeconds(bucketDownWait)
                .build();
        TrajectorySequence secondSample = drive.trajectorySequenceBuilder(bucketPose)
                .lineToSplineHeading(secondSamplePose)
                .addTemporalMarker(() -> {
                    transport.setRot(.2);
                    transport.setExtendoTarget(Ex);
//                    transport.setIntakePower(-1);
                })
                .waitSeconds(intakeWait)
                .addTemporalMarker(() -> {
                    transport.setRot(.925);
                    transport.setExtendoTarget(0);
//                    transport.setIntakePower(0);
                })
                .waitSeconds(retractWait)
                .build();
        TrajectorySequence secondBucket = drive.trajectorySequenceBuilder(secondSamplePose)
                .lineToSplineHeading(bucketPose)
                .UNSTABLE_addTemporalMarkerOffset(bucketOffset, () -> {
//                    transport.setIntakePower(.7);
                })
                .waitSeconds(transferWait)
                .addTemporalMarker(() -> {
                    transport.setOutTarget(3000);
                })
                .waitSeconds(bucketUpWait)
                .addTemporalMarker(() -> {
                    transport.setBucketPitch(.6);
                })
                .waitSeconds(scoreWait)
                .addTemporalMarker(() -> {
                    transport.setBucketPitch(.15);
                    transport.setOutTarget(0);
                })
                .waitSeconds(bucketDownWait)
                .build();
        TrajectorySequence thirdSample = drive.trajectorySequenceBuilder(bucketPose)
                .lineToSplineHeading(thirdSamplePose)
                .addTemporalMarker(() -> {
                    transport.setRot(.2);
                    transport.setExtendoTarget(Ex);
//                    transport.setIntakePower(-1);
                })
                .waitSeconds(intakeWait)
                .addTemporalMarker(() -> {
                    transport.setRot(.925);
                    transport.setExtendoTarget(0);
//                    transport.setIntakePower(0);
                })
                .waitSeconds(retractWait)
                .build();
        TrajectorySequence thirdBucket = drive.trajectorySequenceBuilder(thirdSamplePose)
                .lineToSplineHeading(bucketPose)
                .UNSTABLE_addTemporalMarkerOffset(bucketOffset, () -> {
//                    transport.setIntakePower(.7);
                })
                .waitSeconds(transferWait)
                .addTemporalMarker(() -> {
                    transport.setOutTarget(3000);
                })
                .waitSeconds(bucketUpWait)
                .addTemporalMarker(() -> {
                    transport.setBucketPitch(.6);
                })
                .waitSeconds(scoreWait)
                .addTemporalMarker(() -> {
                    transport.setBucketPitch(.15);
                    transport.setOutTarget(0);
                })
                .waitSeconds(bucketDownWait)
                .build();



//
//        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(BlueClose.blueLeftBoard)
//                .waitSeconds(2)
//                .addTemporalMarker(()-> {transport.closeIntaking();})
//                .lineToSplineHeading(BlueClose.blueLeftClose)
//                .addTemporalMarker(()->{
//                   transport.fullRightClaw();
//                })
//                .build();
//
//        TrajectorySequence centerBoard = drive.trajectorySequenceBuilder(startPose)
////                .addTemporalMarker(()->{transport.closeIntaking();})
////                .waitSeconds(4)
////                .addTemporalMarker(()->{
////                    transport.fullRightClaw();
////                })
////                .waitSeconds(.2)
////                .addTemporalMarker(()->{transport.two();})
//                .lineToSplineHeading(BlueClose.blueCenterBoard)
////                .addTemporalMarker(()->{
////                        transport.fullLeftClaw();
////                })
////                .addTemporalMarker(()->{transport.reset();})
//                .build();
//
//        TrajectorySequence centerPixel = drive.trajectorySequenceBuilder(BlueClose.blueCenterBoard)
//                .waitSeconds(2)
//                //                .addTemporalMarker(()-> {transport.closeIntaking();})
//                .lineToSplineHeading(BlueClose.blueCenterClose)
////                .addTemporalMarker(()->{
////                   transport.fullRightClaw();
////                })
//                .build();
//
//        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(startPose)
////                .UNSTABLE_addTemporalMarkerOffset(1, () -> {transport.two();})
//                .lineToSplineHeading(BlueClose.blueRightBoard)
////                .addTemporalMarker(()->{
////                    transport.fullLeftClaw();
////                })
////                .waitSeconds(.2)
////                .addTemporalMarker(()->{transport.farIntaking();})
////                .waitSeconds(4)
////                .addTemporalMarker(()->{
////                    transport.fullRightClaw();
////                })
////                .waitSeconds(.2)
////                .addTemporalMarker(()->{
////                    transport.closeIntaking();
////                })
//                .build();
//
//        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(BlueClose.blueRightBoard)
//                .waitSeconds(2)
//                //                .addTemporalMarker(()-> {transport.closeIntaking();})
//                .lineToSplineHeading(BlueClose.blueRightClose)
////                .addTemporalMarker(()->{
////                   transport.fullRightClaw();
////                })
//                .build();

        while (opModeInInit() && !isStopRequested()) {
//            BlueClose.vision();
//            telemetry.addData("Spike Pos", BlueClose.spikePos);
//            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive() && !isStopRequested()) {
            followTrajectory(specimen);
            followTrajectory(firstSample);
            followTrajectory(firstbucket);
            followTrajectory(secondSample);
            followTrajectory(secondBucket);
            followTrajectory(thirdSample);
            followTrajectory(thirdBucket);
//            switch (BlueClose.spikePos) {
//                case LEFT:
//                    followTrajectory(leftBoard);
//                    followTrajectory(BlueClose.LeftBoardPark(true));
//                    sleep(30000);
//                case CENTER:
//                    followTrajectory(centerBoard);
//                    followTrajectory(BlueClose.LeftBoardPark(true));
//                    sleep(30000);
//                case RIGHT:
//                    followTrajectory(rightBoard);
//                    followTrajectory(BlueClose.LeftBoardPark(true));
//                    sleep(30000);
//            }

            //TODO: Test All Trajectories:
            //Travel to Board First
//            followTrajectory(leftBoard);
//            followTrajectory(centerBoard);
//            followTrajectory(firstSample);
//            followTrajectory(bucket1);
//            followTrajectory(mainSample);
//            followTrajectory(bucket2);
//            followTrajectory(mainSample);
//            followTrajectory(bucket2);
            //Traveling to Spike Mark Second
//            followTrajectory(leftPixel);
//            followTrajectory(centerPixel);
//            followTrajectory(rightPixel);
            //Parking Last
            //Parking from Left Spike
//            followTrajectory(BlueClose.LeftPark(true));
//            followTrajectory(BlueClose.LeftPark(false));
            //Parking from Center Spike
//            followTrajectory(BlueClose.CenterPark(true));
//            followTrajectory(BlueClose.CenterPark(false));
            //Parking from RightSpike
//            followTrajectory(BlueClose.RightPark(true));
//            followTrajectory(BlueClose.RightBoardToLeftStack(false));
//            followTrajectory(BlueClose.RightBoardToLeftStack(true));
//            followTrajectory(BlueClose.RightPark(false));
        }
    }
}
