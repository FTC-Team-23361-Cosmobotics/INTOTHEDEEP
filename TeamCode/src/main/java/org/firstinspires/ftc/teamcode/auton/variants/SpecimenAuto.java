package org.firstinspires.ftc.teamcode.auton.variants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.transport.TransportFSM;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.transport.EncoderStorage;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
@Config
public class SpecimenAuto extends LinearOpMode {
    public TransportFSM transport;
    public SampleMecanumDrive drive;
    public TrajectorySequence specimen;
    public static Pose2d obsvPosThree, offsetWallPose, startPose, obsvSevenPre, obsvPreThree, obsvPreInter, obsvFivePre, obsvSixPre, obsvPostThree, obsvPostFour, wallPose, specPose, specPoseTwo, specPoseThree, specPoseFour, specPoseFive, obsvInter, obsvOnePre, obsvOnePost, obsvTwoPre, obsvTwoPost;


    public static double startX = 28, startY = -64, startHeading = Math.toRadians(90);
    public static double wallX = 38, wallY = -65.5, wallHeading = Math.toRadians(90), specToWallSpline = Math.toRadians(-90);
    public static double specX = 1, specY = -32, specHeading = Math.toRadians(90), wallToSpecSpline = Math.toRadians(180);
    public static double obsvPreInterX = 8, obsvPreInterY = -36, obsvPreInterHeading = Math.toRadians(90);

    public static double obsvInterX = 37, obsvInterY = -32, obsvInterHeading = Math.toRadians(90), obsvInterSpline = Math.toRadians(75);
    public static double obsvOnePreX = 45, obsvOnePreY = -10, obsvOnePreHeading = Math.toRadians(90), obsvOnePreSpline = Math.toRadians(5);
    public static double obsvTwoPreX = 48, obsvTwoPreY = -25, obsvTwoPreHeading = Math.toRadians(90), obsvTwoPreSpline = Math.toRadians(-95);
    public static double obsvOnePostX = 48, obsvOnePostY = -44, obsvOnePostHeading = Math.toRadians(90), obsvOnePostSpline = Math.toRadians(-95);

    public static double obsvThreePreX = 50, obsvThreePreY = -20, obsvThreePreHeading = Math.toRadians(90);
    public static double obsvPreThreeX = 58, obsvPreThreeY = -15, obsvPreThreeHeading = Math.toRadians(90), obsvThreePreSpline = Math.toRadians(5);
    public static double obsvFourPreX = 62, obsvFourPreY = -30, obsvFourPreHeading = Math.toRadians(90), obsvFourPreSpline = Math.toRadians(-95);
    public static double obsvTwoPostX = 63, obsvTwoPostY = -44, obsvTwoPostHeading = Math.toRadians(90), obsvTwoPostSpline = Math.toRadians(-95);

    public static double obsvFivePreX = 63, obsvFivePreY = -20, obsvFivePreHeading = Math.toRadians(90);
    public static double obsvSixPreX = 56, obsvSixPreY = -15, obsvSixPreHeading = Math.toRadians(90), obsvSixPreSpline = Math.toRadians(-5);
    public static double obsvSevenPreX = 61, obsvSevenPreY = -10, obsvSevenPreHeading = Math.toRadians(90), obsvSevenPreSpline = Math.toRadians(5);
    public static double obsvThreePostX = 62, obsvThreePostY = -43, obsvThreePostHeading = Math.toRadians(90), obsvThreePostSpline = Math.toRadians(-90);
    public static double offsetWallY = -66;
    public static double specUpWait = .5;
    public static double specOpenWaitFirst = 1.75;
    public static double specOpenWait = 1.85;
    public static double firstObsvWait = .5;
    public static double retractFlickWait = .5;
    public static double secondObsvWait = .1;
    public static double thirdObsvWait = .6;
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
        startPose = new Pose2d(startX, startY, startHeading);
        offsetWallPose = new Pose2d(wallX, offsetWallY, wallHeading);
        wallPose = new Pose2d(wallX, wallY, wallHeading);
        specPose = new Pose2d(specX, specY, specHeading);
        specPoseTwo = new Pose2d(specX - 1.5, specY, specHeading);
        specPoseThree = new Pose2d(specX - 1, specY, specHeading);
        obsvPostThree = new Pose2d(obsvThreePreX, obsvThreePreY, obsvThreePreHeading);
        obsvPostFour = new Pose2d(obsvFourPreX, obsvFourPreY, obsvFourPreHeading);
        specPoseFour = new Pose2d(specX - 1, specY, specHeading);
        specPoseFive = new Pose2d(specX - 1, specY, specHeading);
        obsvInter = new Pose2d(obsvInterX, obsvInterY, obsvInterHeading);
        obsvOnePre = new Pose2d(obsvOnePreX, obsvOnePreY, obsvOnePreHeading);
        obsvOnePost = new Pose2d(obsvOnePostX, obsvOnePostY, obsvOnePostHeading);
        obsvTwoPre = new Pose2d(obsvTwoPreX, obsvTwoPreY, obsvTwoPreHeading);
        obsvTwoPost = new Pose2d(obsvTwoPostX, obsvTwoPostY, obsvTwoPostHeading);
        obsvFivePre = new Pose2d(obsvFivePreX, obsvFivePreY, obsvFivePreHeading);
        obsvSixPre = new Pose2d(obsvSixPreX, obsvSixPreY, obsvSixPreHeading);
        obsvSevenPre = new Pose2d(obsvSevenPreX, obsvSevenPreY, obsvSevenPreHeading);
        obsvPreInter = new Pose2d(obsvPreInterX, obsvPreInterY, obsvPreInterHeading);
        obsvPreThree = new Pose2d(obsvPreThreeX, obsvPreThreeY, obsvPreThreeHeading);
        obsvPosThree = new Pose2d(obsvThreePostX, obsvThreePostY, obsvThreePostHeading);
        drive.setPoseEstimate(startPose);

        //Board Auton:
        specimen = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(specUpWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                })
                .UNSTABLE_addTemporalMarkerOffset(specUpWait + TransportFSM.specRotWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWaitFirst, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait + .75, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait - .25 + TransportFSM.specScoreWait - .25, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                })
                .lineToLinearHeading(specPose)
                .lineToLinearHeading(obsvPreInter)
                .splineToLinearHeading(obsvInter, obsvInterSpline)
                .splineToLinearHeading(obsvOnePre, obsvOnePreSpline)
                .splineToLinearHeading(obsvTwoPre, obsvTwoPreSpline)
                .splineToLinearHeading(obsvOnePost, obsvOnePostSpline)
                .lineToLinearHeading(obsvPostThree)
                .splineToLinearHeading(obsvPreThree, obsvThreePreSpline)
                .splineToLinearHeading(obsvPostFour, obsvFourPreSpline)
                .splineToLinearHeading(obsvTwoPost, obsvTwoPostSpline)
                .lineToLinearHeading(obsvFivePre)
                .splineToLinearHeading(obsvSixPre, obsvSixPreSpline)
                .splineToLinearHeading(obsvSevenPre, obsvSevenPreSpline)
                .lineToLinearHeading(obsvPosThree)
                .splineToLinearHeading(offsetWallPose, specToWallSpline)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                })
                .UNSTABLE_addTemporalMarkerOffset(specUpWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                })
                .UNSTABLE_addTemporalMarkerOffset(specUpWait + TransportFSM.specRotWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait + TransportFSM.specScoreWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                })
                .lineToLinearHeading(specPoseTwo)
                .lineToLinearHeading(wallPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                })
                .UNSTABLE_addTemporalMarkerOffset(specUpWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                })
                .UNSTABLE_addTemporalMarkerOffset(specUpWait + TransportFSM.specRotWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait + TransportFSM.specScoreWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                })
                .lineToLinearHeading(specPoseThree)
                .lineToLinearHeading(wallPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                })
                .UNSTABLE_addTemporalMarkerOffset(specUpWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                })
                .UNSTABLE_addTemporalMarkerOffset(specUpWait + TransportFSM.specRotWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait + TransportFSM.specScoreWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                })
                .lineToLinearHeading(specPoseFour)
                .lineToLinearHeading(wallPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                })
                .UNSTABLE_addTemporalMarkerOffset(specUpWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                })
                .UNSTABLE_addTemporalMarkerOffset(specUpWait + TransportFSM.specRotWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                })
                .UNSTABLE_addTemporalMarkerOffset(specOpenWait + TransportFSM.specRetractWait + TransportFSM.specScoreWait, () -> {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                })
                .lineToLinearHeading(specPoseFive)
                .lineToLinearHeading(wallPose)
                .build();

        TransportFSM.isSpec = true;

        while (opModeInInit() && !isStopRequested()) {
            //TODO: telemetry
            //TODO: cll resetPosIMU(), possibly implementing something to not cll infinitely

            EncoderStorage.isAuto = true;
            transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
            transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
            transport.flickerOut(false);
        }

        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive() && !isStopRequested()) {
            followTrajectory(specimen);
        }
    }
}

