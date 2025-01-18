package org.firstinspires.ftc.teamcode.auton.variants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.transport.TransportFSM;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
@Config
public class SampleAuto extends LinearOpMode {
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
        drive = new SampleMecanumDrive(hardwareMap);
        transport = new TransportFSM(hardwareMap);
        StartPose = new Pose2d(-22, -60, Math.toRadians(0));
        firstBucketPose = new Pose2d(FirstBucketX, FirstBucketY, FirstBucketHeading);
        bucketPose = new Pose2d(BucketX, BucketY, BucketHeading);
        firstSamplePose  = new Pose2d(FirstSampleX, FirstSampleY, FirstSampleHeading);
        secondSamplePose = new Pose2d(SecondSampleX, SecondSampleY, SecondSampleHeading);
        thirdSamplePose  = new Pose2d(ThirdSampleX, ThirdSampleY, ThirdSampleHeading);
        thirdBucketPose = new Pose2d(ThirdBucketX, ThirdBucketY, ThirdBucketHeading);
        parkPose = new Pose2d(parkX, parkY, parkHeading);
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

        TransportFSM.isSpec = false;

        while (opModeInInit() && !isStopRequested()) {
            //TODO: TELEMETRY
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
        }
    }
}
