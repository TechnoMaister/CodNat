package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "Aurora")
public class Aurora extends LinearOpMode {
    OpenCvCamera camera;
    Pipelinee aprilTagDetectionPipeline;

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    private DcMotorEx motorKatanaDreapta = null;
    private DcMotorEx motorKatanaStanga = null;
    private DcMotor motorColector = null;

    private Servo servoGhearaStanga = null;
    private Servo servoGhearaDreapta = null;

    double numarRotatii = 1440;
    double diametruRoata = 9.9822;
    double rotatiiPeCM = numarRotatii / (diametruRoata * Math.PI);

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int stanga = 14;
    int mijloc = 15;
    int dreapta = 13;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        hardwareMap();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new Pipelinee(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == stanga || tag.id == mijloc || tag.id == dreapta)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound){
                    if(tagOfInterest.id == stanga)
                        telemetry.addLine("Pozitia: Stanga");
                    if(tagOfInterest.id == mijloc)
                        telemetry.addLine("Pozitia: Mijloc");
                    if(tagOfInterest.id == dreapta)
                        telemetry.addLine("Pozitia: Dreapta");
                } else
                    telemetry.addLine("Nu vad");

            }

            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest == null){
            Drive(0.5,-40,40,40,-40);
        }else if(tagOfInterest.id == stanga){
            Drive(0.5,-34,-34,-34,-34);
            Drive(0.5,-36,36,36,-36);
        }else if(tagOfInterest.id == mijloc){
            Drive(0.5,-40,40,40,-40);
        }else if(tagOfInterest.id == dreapta){
            Drive(0.5,32,32,32,32);
            Drive(0.5,-36,36,36,-36);
        }
    }

    void hardwareMap(){

        backLeft = hardwareMap.get(DcMotor.class, "bl");
        frontLeft = hardwareMap.get(DcMotor.class,"fl");
        frontRight = hardwareMap.get(DcMotor.class,"fr");
        backRight = hardwareMap.get(DcMotor.class,"br");

        motorKatanaDreapta  = hardwareMap.get(DcMotorEx.class, "motorKatanaDreapta");
        motorKatanaStanga  = hardwareMap.get(DcMotorEx.class, "motorKatanaStanga");
        motorColector  = hardwareMap.get(DcMotorEx.class, "colector");

        servoGhearaStanga = hardwareMap.get(Servo.class, "ghearaStanga");
        servoGhearaDreapta = hardwareMap.get(Servo.class, "ghearaDreapta");

        motorKatanaDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorKatanaStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        motorKatanaDreapta.setDirection(DcMotorEx.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void Drive(double viteza, double fataStangaDistanta, double fataDreaptaDistanta, double spateStangaDistanta, double spateDreaptaDistanta) {
        int frontLeftPozitia;
        int frontRightPozitie;
        int backLeftPozitie;
        int backRightPozitie;

        if (opModeIsActive()) {

            frontLeftPozitia  = frontLeft.getCurrentPosition() + (int) (fataStangaDistanta * rotatiiPeCM);
            frontRightPozitie = frontRight.getCurrentPosition() + (int) (fataDreaptaDistanta * rotatiiPeCM);
            backLeftPozitie   = backLeft.getCurrentPosition() + (int) (spateStangaDistanta * rotatiiPeCM);
            backRightPozitie  = backRight.getCurrentPosition() + (int) (spateDreaptaDistanta * rotatiiPeCM);

            frontLeft.setTargetPosition(frontLeftPozitia);
            frontRight.setTargetPosition(frontRightPozitie);
            backLeft.setTargetPosition(backLeftPozitie);
            backRight.setTargetPosition(backRightPozitie);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(viteza);
            frontRight.setPower(viteza);
            backLeft.setPower(viteza);
            backRight.setPower(viteza);

            while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}