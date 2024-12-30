package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {

        double BucketX = -50;
        double BucketY = -60;
        double BucketHeading = Math.toRadians(0);
        double FirstSampleX = -44;
        double FirstSampleY = -60;
        double FirstSampleHeading = Math.toRadians(0);
        double SecondSampleX = -55;
        double SecondSampleY = -35;
        double SecondSampleHeading = 1.57;
        double ThirdSampleX = -50;
        double ThirdSampleY = 0;
        double ThirdSampleHeading = 3.14;
        double parkX = 0;
        double parkY = 0;
        double parkHeading = 0;
//        Pose2d StartBlueBucketPose = new Pose2d(22, 60, Math.toRadians(270));
//        Pose2d StartBlueObsPose = new Pose2d(-22, 60, Math.toRadians(270));
//        Pose2d StartRedObsPose = new Pose2d(22, -60, Math.toRadians(90));
        Pose2d StartPose = new Pose2d(-22, -60, Math.toRadians(0));
        Pose2d bucketPose = new Pose2d(BucketX, BucketY, BucketHeading);
        Pose2d firstSamplePose  = new Pose2d(FirstSampleX, FirstSampleY, FirstSampleHeading);
        Pose2d secondSamplePose = new Pose2d(SecondSampleX, SecondSampleY, SecondSampleHeading);
        Pose2d thirdSamplePose  = new Pose2d(ThirdSampleX, ThirdSampleY, ThirdSampleHeading);
        Pose2d parkPose = new Pose2d(parkX, parkY, parkHeading);


        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.646780560809674, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-55, -56, Math.toRadians(45)))
                        .splineToSplineHeading(new Pose2d(-30, -10, 3.1415), Math.toRadians(-15))
                        .splineToLinearHeading(new Pose2d(-20, -10, 3.1415), Math.toRadians(0))
//                        .waitSeconds(0.5)
//                        //.lineTo(new Vector2d(-34,-36))
//                        .splineToSplineHeading(new Pose2d( -25, -34, Math.toRadians(160)), Math.toRadians(135))
//                        .waitSeconds(0.5)
//                        .lineToSplineHeading(new Pose2d(-52,-53, Math.toRadians(45)))
//                        .waitSeconds(0.5)
//                        .lineToSplineHeading(new Pose2d( -38, -25, Math.toRadians(180)))
//                        .waitSeconds(0.5)
//                        .lineToSplineHeading(new Pose2d(-52,-53, Math.toRadians(45)))
//                        .waitSeconds(0.5)
//                        .lineToSplineHeading(new Pose2d( -38, -25, Math.toRadians(180)))
//                        .waitSeconds(0.5)
//                        .lineToSplineHeading(new Pose2d(-52,-53, Math.toRadians(45)))
//                        .waitSeconds(0.5)

                        /*
                        .lineToSplineHeading(new Pose2d(-35,-25, Math.toRadians(180)))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(-52,-53, Math.toRadians(45)))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(-45,-25, Math.toRadians(180)))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(-52,-53, Math.toRadians(45)))
                        .waitSeconds(0.5)*/
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}