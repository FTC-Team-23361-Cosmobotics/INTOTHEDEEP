//package org.firstinspires.ftc.teamcode.auton.variants;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.auton.Auton;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.teleop.AllianceStorage;
//
//@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
//public class RedClose extends LinearOpMode {
//    Auton RedClose;
//    Transport transport;
//    SampleMecanumDrive drive;
//
//    public void followTrajectory(TrajectorySequence traj) {
//        drive.followTrajectorySequenceAsync(traj);
//        while (!isStopRequested() && drive.isBusy()) {
//            drive.update();
//            transport.update();
//        }
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        RedClose = new Auton(hardwareMap);
//        transport = RedClose.transport();
//        drive = RedClose.drive();
//        AllianceStorage.isRed = true;
//        RedClose.isLeftClaw = true; //YELLOW PIXEL IN THIS CLAW
//        Pose2d startPose = new Pose2d(15.25, -63.25, Math.toRadians(90));
//        drive.setPoseEstimate(startPose);
//
//        //Board Auton:
//        TrajectorySequence leftBoard = drive.trajectorySequenceBuilder(startPose)
////                .UNSTABLE_addTemporalMarkerOffset(2, () -> {transport.two();})
//                .lineToSplineHeading(RedClose.redLeftBoard)
////                .addTemporalMarker(() -> {
//////                    if (BlueClose.leftClaw()) {
//////                        BlueClose.fullLeftClaw();
//////                    } else {
//////                        BlueClose.fullRightClaw();
//////                    }
////                    transport.fullLeftClaw();
////                })
//                .build();
//
//
//        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(RedClose.redLeftBoard)
//                .waitSeconds(2)
//                //                .addTemporalMarker(()-> {transport.closeIntaking();})
//                .lineToSplineHeading(RedClose.redLeftClose)
////                .addTemporalMarker(()->{
////                   transport.fullRightClaw();
////                })
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
//                .lineToSplineHeading(RedClose.redCenterBoard)
////                .addTemporalMarker(()->{
////                        transport.fullLeftClaw();
////                })
////                .addTemporalMarker(()->{transport.reset();})
//                .build();
//
//        TrajectorySequence centerPixel = drive.trajectorySequenceBuilder(RedClose.redCenterBoard)
//                .waitSeconds(2)
//                //                .addTemporalMarker(()-> {transport.closeIntaking();})
//                .lineToSplineHeading(RedClose.redCenterClose)
////                .addTemporalMarker(()->{
////                   transport.fullRightClaw();
////                })
//                .build();
//
//        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(startPose)
////                .UNSTABLE_addTemporalMarkerOffset(1, () -> {transport.two();})
//                .lineToSplineHeading(RedClose.redRightBoard)
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
//        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(RedClose.redRightBoard)
//                .waitSeconds(2)
//                //                .addTemporalMarker(()-> {transport.closeIntaking();})
//                .lineToSplineHeading(RedClose.redRightClose)
////                .addTemporalMarker(()->{
////                   transport.fullRightClaw();
////                })
//                .build();
//
//        while (opModeInInit() && !isStopRequested()) {
//            RedClose.vision();
//            telemetry.addData("Spike Pos", RedClose.spikePos);
//            telemetry.update();
//        }
//        waitForStart();
//        if (isStopRequested()) return;
//        if (opModeIsActive() && !isStopRequested()) {
////            switch (RedClose.spikePos) {
////                case LEFT:
////                    followTrajectory(leftBoard);
////                    followTrajectory(RedClose.LeftBoardPark(true));
////                    sleep(30000);
////                case CENTER:
////                    followTrajectory(centerBoard);
////                    followTrajectory(RedClose.LeftBoardPark(true));
////                    sleep(30000);
////                case RIGHT:
////                    followTrajectory(rightBoard);
////                    followTrajectory(RedClose.LeftBoardPark(true));
////                    sleep(30000);
////            }
//
//            //TODO: Test All Trajectories:
//            //Travel to Board First
//            followTrajectory(centerBoard);
////            followTrajectory(centerBoard);
////            followTrajectory(rightBoard);
//            //Traveling to Spike Mark Second
//            followTrajectory(centerPixel);
////            followTrajectory(centerPixel);
////            followTrajectory(rightPixel);
//            //Parking Last
//            //Parking from Left Spike
////            followTrajectory(RedClose.LeftBoardToRightStack(false));
////            followTrajectory(RedClose.CenterBoardToLeftStack(true));
//            followTrajectory(RedClose.CenterPark(false));
////            followTrajectory(RedClose.LeftPark(false));
//            //Parking from Center Spike
////            followTrajectory(RedClose.CenterPark(true));
////            followTrajectory(RedClose.CenterPark(false));
//            //Parking from RightSpike
////            followTrajectory(RedClose.RightPark(true));
////            followTrajectory(RedClose.RightPark(false));
//        }
//    }
//}

