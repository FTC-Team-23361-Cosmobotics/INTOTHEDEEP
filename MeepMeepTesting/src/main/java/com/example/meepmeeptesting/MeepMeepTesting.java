package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Pose2d StartBlueBucketPose = new Pose2d(22, 60, Math.toRadians(270));
        Pose2d StartRedBucketPose = new Pose2d(-22, -60, Math.toRadians(270));
        Pose2d StartBlueObsPose = new Pose2d(-22, 60, Math.toRadians(270));
        Pose2d StartRedObsPose = new Pose2d(22, -60, Math.toRadians(90));

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(StartRedBucketPose)
                        .lineToSplineHeading(new Pose2d(-9, -35, Math.toRadians(270)))
                        .waitSeconds(0.5)
                        //.lineTo(new Vector2d(-34,-36))
                        .splineToSplineHeading(new Pose2d( -25, -34, Math.toRadians(160)), Math.toRadians(135))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(-52,-53, Math.toRadians(45)))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d( -38, -25, Math.toRadians(180)))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(-52,-53, Math.toRadians(45)))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d( -38, -25, Math.toRadians(180)))
                        .waitSeconds(0.5)
                        .lineToSplineHeading(new Pose2d(-52,-53, Math.toRadians(45)))
                        .waitSeconds(0.5)

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