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
//public class BlueClose extends LinearOpMode {
//    Auton BlueClose;
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
//    @Override
//    public void runOpMode() throws InterruptedException {
//        BlueClose = new Auton(hardwareMap);
//        transport = BlueClose.transport();
//        drive = BlueClose.drive();
//        AllianceStorage.isRed = false;
//        BlueClose.isLeftClaw = true; //YELLOW PIXEL IN THIS CLAW
//        Pose2d startPose = new Pose2d(15.25,63.25, Math.toRadians(270));
//        drive.setPoseEstimate(startPose);
//
//        //Board Auton:
//        TrajectorySequence leftBoard = drive.trajectorySequenceBuilder(startPose)
//                .UNSTABLE_addTemporalMarkerOffset(2, () -> {transport.one();})
//                .lineToSplineHeading(BlueClose.blueLeftBoard)
//                .addTemporalMarker(() -> {
//////                    if (BlueClose.leftClaw()) {
//////                        BlueClose.fullLeftClaw();
//////                    } else {
//////                        BlueClose.fullRightClaw();
//////                    }
//                    transport.fullLeftClaw();
//                })
//                .build();
//
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
//
//        while (opModeInInit() && !isStopRequested()) {
//            BlueClose.vision();
//            telemetry.addData("Spike Pos", BlueClose.spikePos);
//            telemetry.update();
//        }
//
//        waitForStart();
//        if (isStopRequested()) return;
//        if (opModeIsActive() && !isStopRequested()) {
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
//            followTrajectory(rightBoard);
//            //Traveling to Spike Mark Second
////            followTrajectory(leftPixel);
////            followTrajectory(centerPixel);
//            followTrajectory(rightPixel);
//            //Parking Last
//            //Parking from Left Spike
////            followTrajectory(BlueClose.LeftPark(true));
////            followTrajectory(BlueClose.LeftPark(false));
//            //Parking from Center Spike
////            followTrajectory(BlueClose.CenterPark(true));
////            followTrajectory(BlueClose.CenterPark(false));
//            //Parking from RightSpike
////            followTrajectory(BlueClose.RightPark(true));
//            followTrajectory(BlueClose.RightBoardToLeftStack(false));
//            followTrajectory(BlueClose.RightBoardToLeftStack(true));
//            followTrajectory(BlueClose.RightPark(false));
//        }
//    }
//}
