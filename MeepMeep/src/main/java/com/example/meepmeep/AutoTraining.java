package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoTraining {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, -60, Math.PI/2))
                .splineToLinearHeading(new Pose2d(-48,-48,-Math.PI*3/4), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-46,-37,Math.PI/2), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-48,-48,-Math.PI*3/4), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-56,-37,Math.PI/2), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-48,-48,-Math.PI*3/4), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-66,-37,Math.PI/2), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-48,-48,-Math.PI*3/4), Math.PI/2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
