package com.mercury.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTest {
    public static void main(String[] args) {
        final double QUARTER_FIELD_IN = 35.275;
        final double ROBOT_WIDTH_IN = 15;
        final double STARTING_Y_IN = 3.0 * ROBOT_WIDTH_IN - 2.0 * (QUARTER_FIELD_IN + 18.0);

        final TrajectoryVelocityConstraint CONSTRAINT = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(30.0), new AngularVelocityConstraint(1.0)
        ));

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(
                    new Pose2d())
                            .lineTo(new Vector2d(-12,
                                            36)).build());

/*
                    new Pose2d(-QUARTER_FIELD_IN, STARTING_Y_IN, Math.toRadians(90.0)))
                               .setVelConstraint(CONSTRAINT)
                               .splineToConstantHeading(new Vector2d(-QUARTER_FIELD_IN - ROBOT_WIDTH_IN * 0.75,
                                                                     -QUARTER_FIELD_IN - ROBOT_WIDTH_IN * 0.6),
                                                            Math.toRadians(180))
                               .waitSeconds(0.5)
                               .build());
 */

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
            .setDarkMode(true)
            .addEntity(myBot)
            .start();
    }
}