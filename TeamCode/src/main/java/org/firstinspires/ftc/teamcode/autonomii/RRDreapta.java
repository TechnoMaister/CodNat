package org.firstinspires.ftc.teamcode.autonomii;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

@Config
@Autonomous(name = "RRDreapta")
public class RRDreapta extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start = new Pose2d(35, 60, 0);

        TrajectorySequence traiectorie = drive.trajectorySequenceBuilder(start)
                // inchideGheara();
                // ridica(2900);

                .back(30)
                .strafeLeft(56)
                .forward(8)
                // deschideGheara();
                .back(8)
                //ridica(600);
                .strafeLeft(15)
                //ridica(600);
                .forward(40)
                .forward(5)

                .back(5)
                .back(40)
                //inchideGheara();
                //ridica(2900);
                //motorColector.setPower(1);
                .strafeRight(12)
                .forward(5)
                //deschideGheara();
                .back(5)
                .strafeLeft(12)
                .forward(25)
                .build();

        waitForStart();
        drive.followTrajectorySequence(traiectorie);
    }
}
