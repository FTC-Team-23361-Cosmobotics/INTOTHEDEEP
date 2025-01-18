package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.SampleMecanumDrive;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        double startX = 0, startY = -60, startHeading = Math.toRadians(90);
        double wallX = 35, wallY = -60, wallHeading = Math.toRadians(90), specToWallSpline = Math.toRadians(-90);
        double specX = 1.5, specY = -32, specHeading = Math.toRadians(90), wallToSpecSpline = Math.toRadians(180);
        double obsvPreInterX = 8, obsvPreInterY = -36, obsvPreInterHeading = Math.toRadians(90);

        double obsvInterX = 35, obsvInterY = -32, obsvInterHeading = Math.toRadians(90), obsvInterSpline = Math.toRadians(75);
        double obsvOnePreX = 41, obsvOnePreY = -10, obsvOnePreHeading = Math.toRadians(90), obsvOnePreSpline = Math.toRadians(5);
        double obsvTwoPreX = 46, obsvTwoPreY = -25, obsvTwoPreHeading = Math.toRadians(90), obsvTwoPreSpline = Math.toRadians(-95);
        double obsvOnePostX = 46, obsvOnePostY = -45, obsvOnePostHeading = Math.toRadians(90), obsvOnePostSpline = Math.toRadians(-95);

        double obsvThreePreX = 46, obsvThreePreY = -15, obsvThreePreHeading = Math.toRadians(90);
        double obsvPreThreeX = 51, obsvPreThreeY = -10, obsvPreThreeHeading = Math.toRadians(90), obsvThreePreSpline = Math.toRadians(5);
        double obsvFourPreX = 57, obsvFourPreY = -25, obsvFourPreHeading = Math.toRadians(90), obsvFourPreSpline = Math.toRadians(-95);
        double obsvTwoPostX = 57, obsvTwoPostY = -45, obsvTwoPostHeading = Math.toRadians(90), obsvTwoPostSpline = Math.toRadians(-95);

        double obsvFivePreX = 57, obsvFivePreY = -15, obsvFivePreHeading = Math.toRadians(90);
        double obsvSixPreX = 62, obsvSixPreY = -10, obsvSixPreHeading = Math.toRadians(90), obsvSixPreSpline = Math.toRadians(5);
        double obsvSevenPreX = 62, obsvSevenPreY = -40, obsvSevenPreHeading = Math.toRadians(90), obsvSevenPreSpline = Math.toRadians(-95);
        double offsetWallX = 62;

        Pose2d startPose, obsvSevenPre, obsvPreThree, obsvPreInter, obsvFivePre, obsvSixPre, obsvPostThree, obsvPostFour, wallPose, specPose, specPoseTwo, specPoseThree, specPoseFour, specPoseFive, obsvInter, obsvOnePre, obsvOnePost, obsvTwoPre, obsvTwoPost;

        startPose = new Pose2d(startX, startY, startHeading);
        wallPose = new Pose2d(wallX, wallY, wallHeading);
        specPose = new Pose2d(specX, specY, specHeading);
        specPoseTwo = new Pose2d(specX + .5, specY, specHeading);
        specPoseThree = new Pose2d(specX + 1, specY, specHeading);
        obsvPostThree = new Pose2d(obsvThreePreX, obsvThreePreY, obsvThreePreHeading);
        obsvPostFour = new Pose2d(obsvFourPreX, obsvFourPreY, obsvFourPreHeading);
        specPoseFour = new Pose2d(specX + 1.5, specY, specHeading);
        specPoseFive = new Pose2d(specX + 2, specY, specHeading);
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

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.646780560809674, 50, 4.787943032245642, Math.toRadians(180), 21)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
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
                        .lineToLinearHeading(obsvSevenPre)
                        .splineToLinearHeading(wallPose, specToWallSpline)
                        .lineToLinearHeading(specPoseTwo)
                        .lineToLinearHeading(wallPose)
                        .lineToLinearHeading(specPoseThree)
                        .lineToLinearHeading(wallPose)
                        .lineToLinearHeading(specPoseFour)
                        .lineToLinearHeading(wallPose)
                        .lineToLinearHeading(specPoseFive)
                        .lineToLinearHeading(wallPose)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}