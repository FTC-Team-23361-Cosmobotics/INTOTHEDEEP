package org.firstinspires.ftc.teamcode.auton.variants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.Auton;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.transport.TransportFSM;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
@Config
public class SampleAuto extends LinearOpMode {
    Auton Auton1;
    TransportFSM transport;
    SampleMecanumDrive drive;
    public Pose2d StartPose, firstBucketPose, parkPose, bucketPose, firstSamplePose, secondSamplePose, thirdSamplePose, thirdBucketPose;

    public static double BucketX = -54, BucketY = -55, BucketHeading = Math.toRadians(45);
    public static double FirstBucketX = -53, FirstBucketY = -60, FirstBucketHeading = Math.toRadians(0);
    public static double FirstSampleX = -52, FirstSampleY = -43, FirstSampleHeading = 1.57;
    public static double SecondSampleX = -62.5, SecondSampleY = -43, SecondSampleHeading = 1.57;
    public static double ThirdSampleX = -47, ThirdSampleY = -20, ThirdSampleHeading = 3.14159;
    public static double ThirdBucketX = -62, ThirdBucketY = -53, ThirdBucketHeading = Math.toRadians(45);
    public static double parkX = -15, parkY = 0, parkHeading = 3.14159, parkSpline = Math.toRadians(-15);
    public static double transferWait = 0.6;
    public static double dumpWait = 1.2;
    public static double resetOffset = .75;
    public static double extendWait = .25;
    public static double intakeWait = -.5;
    public static double retractIntakeWait = .75;
    public static double retractOutWait = 1.75;
    public static double parkPosWait = 1.5;
    public static double retractExtendoWait = 1;
    public void followTrajectory(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
            transport.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        StartPose = new Pose2d(-22, -60, Math.toRadians(0));
        firstBucketPose = new Pose2d(FirstBucketX, FirstBucketY, FirstBucketHeading);
        bucketPose = new Pose2d(BucketX, BucketY, BucketHeading);
        firstSamplePose  = new Pose2d(FirstSampleX, FirstSampleY, FirstSampleHeading);
        secondSamplePose = new Pose2d(SecondSampleX, SecondSampleY, SecondSampleHeading);
        thirdSamplePose  = new Pose2d(ThirdSampleX, ThirdSampleY, ThirdSampleHeading);
        thirdBucketPose = new Pose2d(ThirdBucketX, ThirdBucketY, ThirdBucketHeading);
        parkPose = new Pose2d(parkX, parkY, parkHeading);
        Auton1 = new Auton(hardwareMap);
        transport = Auton1.transport();
        drive = Auton1.drive();
//        AllianceStorage.isRed = false;
//        BlueClose.isLeftClaw = true; //YELLOW PIXEL IN THIS CLAW
        drive.setPoseEstimate(StartPose);

        //Board Auton:
        TrajectorySequence firstbucket = drive.trajectorySequenceBuilder(StartPose)
                .addTemporalMarker(() -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(transferWait, () -> {
            transport.sampleTransport = TransportFSM.SampleTransport.HIGH_BUCKET;
         })
                .UNSTABLE_addTemporalMarkerOffset(dumpWait,() -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.DUMP;
                })
                .lineToSplineHeading(bucketPose)
                .build();
        TrajectorySequence firstSample = drive.trajectorySequenceBuilder(firstBucketPose)
                .UNSTABLE_addTemporalMarkerOffset(resetOffset,() -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(extendWait, () -> {
                    transport.setExtendoTarget(TransportFSM.autoExtendoUpper);
                })
                .lineToLinearHeading(firstSamplePose)
                .UNSTABLE_addTemporalMarkerOffset(intakeWait, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.INTAKE;
                })
                .waitSeconds(retractIntakeWait)
                .build();
        TrajectorySequence secondbucket = drive.trajectorySequenceBuilder(firstSamplePose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.EXTENDED;
                })
                .UNSTABLE_addTemporalMarkerOffset(retractExtendoWait, () -> {
                    transport.setExtendoTarget(-10);
                })
                .lineToSplineHeading(bucketPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.TRANSFER;
                })
                .UNSTABLE_addTemporalMarkerOffset(transferWait, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.HIGH_BUCKET;
                })
                .UNSTABLE_addTemporalMarkerOffset(dumpWait,() -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.DUMP;
                })
                .waitSeconds(retractOutWait)
                .build();
        TrajectorySequence secondSample = drive.trajectorySequenceBuilder(bucketPose)
                .UNSTABLE_addTemporalMarkerOffset(resetOffset,() -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(extendWait, () -> {
                    transport.setExtendoTarget(TransportFSM.autoExtendoUpper);
                })
                .lineToLinearHeading(secondSamplePose)
                .UNSTABLE_addTemporalMarkerOffset(intakeWait, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.INTAKE;
                })
                .waitSeconds(retractIntakeWait)
                .build();
        TrajectorySequence thirdbucket = drive.trajectorySequenceBuilder(secondSamplePose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.EXTENDED;
                })
                .UNSTABLE_addTemporalMarkerOffset(retractExtendoWait, () -> {
                    transport.setExtendoTarget(-10);
                })
                .lineToLinearHeading(bucketPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.TRANSFER;
                })
                .UNSTABLE_addTemporalMarkerOffset(transferWait, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.HIGH_BUCKET;
                })
                .UNSTABLE_addTemporalMarkerOffset(dumpWait,() -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.DUMP;
                })
                .waitSeconds(retractOutWait)
                .build();
        TrajectorySequence thirdSample = drive.trajectorySequenceBuilder(bucketPose)
                .UNSTABLE_addTemporalMarkerOffset(resetOffset,() -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(extendWait, () -> {
                    transport.setExtendoTarget(TransportFSM.autoExtendoUpper);
                })
                .lineToLinearHeading(thirdSamplePose)
                .UNSTABLE_addTemporalMarkerOffset(intakeWait, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.INTAKE;
                })
                .waitSeconds(retractIntakeWait)
                .build();
        TrajectorySequence fourthBucket = drive.trajectorySequenceBuilder(thirdSamplePose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.EXTENDED;
                })
                .UNSTABLE_addTemporalMarkerOffset(retractExtendoWait, () -> {
                    transport.setExtendoTarget(-10);
                })
                .lineToLinearHeading(thirdBucketPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.TRANSFER;
                })
                .UNSTABLE_addTemporalMarkerOffset(transferWait, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.HIGH_BUCKET;
                })
                .UNSTABLE_addTemporalMarkerOffset(dumpWait,() -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.DUMP;
                })
                .waitSeconds(retractOutWait)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(bucketPose)
                .UNSTABLE_addTemporalMarkerOffset(resetOffset,() -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(parkPosWait, () -> {
                    transport.sampleTransport = TransportFSM.SampleTransport.DUMP;
                })
                .splineToSplineHeading(new Pose2d(-30, -10, 3.1415), Math.toRadians(-15))
                .splineToLinearHeading(new Pose2d(-20, -10, 3.1415), Math.toRadians(0))
                .build();
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

        while (opModeInInit() && !isStopRequested()) {
//            BlueClose.vision();
//            telemetry.addData("Spike Pos", BlueClose.spikePos);
//            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive() && !isStopRequested()) {
            followTrajectory(firstbucket);
            followTrajectory(firstSample);
            followTrajectory(secondbucket);
            followTrajectory(secondSample);
            followTrajectory(thirdbucket);
            followTrajectory(thirdSample);
            followTrajectory(fourthBucket);
            followTrajectory(park);


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
