package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeemMeepTest {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(42.62511518370712, 38.110287416570166, Math.toRadians(138.19991297468354), Math.toRadians(138.19991297468354), 15.75)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35,-60 , 0))
                                // inchideGheara();
                                // ridica(2900);

                                .back(25)
                                .strafeLeft(36)
                                .forward(5)
                                // deschideGheara();
                                .back(5)
                                //ridica(600);
                                .strafeLeft(12)
                                //ridica(600);
                                .lineTo(new Vector2d(60, 12))
                                //inchideGheara();
                                //ridica(2900);
                                .lineTo(new Vector2d(10, -12))
                                //motorColector.setPower(1);
                                .strafeRight(12)
                                .forward(5)
                                //deschideGheara();
                                .back(5)
                                .strafeLeft(12)
                                .forward(25)
                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}