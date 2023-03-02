package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "AutoStanga")
public class AutoStanga extends LinearOpMode {
    OpenCvCamera camera;
    Pipelinee aprilTagDetectionPipeline;
    private BNO055IMU imu;

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    private DcMotorEx motorKatanaDreapta = null;
    private DcMotorEx motorKatanaStanga = null;
    private DcMotor motorColector = null;

    private Servo servoGhearaStanga = null;
    private Servo servoGhearaDreapta = null;

    //NormalizedColorSensor senzorLinie;
    private DistanceSensor sensorRange;

    double numarRotatii = 1440;
    double diametruRoata = 9.9822;
    double rotatiiPeCM = numarRotatii / (diametruRoata * 3.1415);

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int stanga = 14;
    int mijloc = 15;
    int dreapta = 13;

    AprilTagDetection tagulDorit = null;


    @Override
    public void runOpMode()
    {
        hardwareMap();

        inchideGheara();
        /*
        if (senzorLinie instanceof SwitchableLight) {
            ((SwitchableLight)senzorLinie).enableLight(true);
        }

        senzorLinie.setGain(15);

         */

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
                boolean gasit = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == stanga || tag.id == mijloc || tag.id == dreapta)
                    {
                        tagulDorit = tag;
                        gasit = true;
                        break;
                    }
                }

                if(gasit){
                    if(tagulDorit.id == stanga)
                        telemetry.addLine("Pozitia: Stanga");
                    if(tagulDorit.id == mijloc)
                        telemetry.addLine("Pozitia: Mijloc");
                    if(tagulDorit.id == dreapta)
                        telemetry.addLine("Pozitia: Dreapta");
                } else
                    telemetry.addLine("Nu vad");

            }

            telemetry.update();
            sleep(20);
        }

        if(tagulDorit == null){
        }else if(tagulDorit.id == stanga){

            inchideGheara();

            ridica(4100);
            sleep(50);

            Drive(1,30,30,30,30);

            Turn(0);
            Turn(0);

            Drive(0.5,-56,56,56,-56);

            Turn(0);
            Turn(0);

            Drive(0.5,7.3,7.3,7.3,7.3);

            deschideGheara();

            Drive(0.5,-7.3,-7.3,-7.3,-7.3);

            Turn(0);
            Turn(0);

            ridica(0);

            Drive(1,20,-20,-20,20);

            Turn(0);
            Turn(0);

            Drive(1,-61,-61,-61,-61);

            Turn(0);
            Turn(0);

            while(motorKatanaDreapta.isBusy() && motorKatanaStanga.isBusy() && opModeIsActive()){
            }

            Turn(0);
            Turn(0);

        }else if(tagulDorit.id == mijloc){

            inchideGheara();

            ridica(4100);
            sleep(50);

            Drive(1,30,30,30,30);

            Turn(0);
            Turn(0);

            Drive(0.5,-56,56,56,-56);

            Turn(0);
            Turn(0);

            Drive(0.5,7.3,7.3,7.3,7.3);

            deschideGheara();

            Drive(0.5,-7.3,-7.3,-7.3,-7.3);

            Turn(0);
            Turn(0);

            ridica(0);

            Drive(1,20,-20,-20,20);

            Turn(0);
            Turn(0);

            Drive(1,-28,-28,-28,-28);

            Turn(0);
            Turn(0);

            while(motorKatanaDreapta.isBusy() && motorKatanaStanga.isBusy() && opModeIsActive()){
            }

            Turn(0);
            Turn(0);

        }else if(tagulDorit.id == dreapta){

            inchideGheara();

            ridica(4100);
            sleep(50);

            Drive(1,30,30,30,30);

            Turn(0);
            Turn(0);

            Drive(0.5,-56,56,56,-56);

            Turn(0);
            Turn(0);

            Drive(0.5,7.3,7.3,7.3,7.3);

            deschideGheara();

            Drive(0.5,-7.3,-7.3,-7.3,-7.3);

            Turn(0);
            Turn(0);

            ridica(0);

            Drive(1,20,-20,-20,20);

            Turn(0);
            Turn(0);

            Drive(1,-1,-1,-1,-1);

            Turn(0);
            Turn(0);

            while(motorKatanaDreapta.isBusy() && motorKatanaStanga.isBusy() && opModeIsActive()){
            }

            Turn(0);
            Turn(0);
        }
    }

    void mersCuGyro(int pozitie, double putere){
        double frontLeftViteza;
        double frontRightViteza;
        double backLeftViteza;
        double backRightViteza;

        double pozitiaDorita = PozitiaActuala();
        double pozitiaInitiala = 0;

        while(frontLeft.getCurrentPosition()< pozitie + pozitiaInitiala) {
            double valoareDeSchimbat = PozitiaActuala();

            frontLeftViteza = putere - (valoareDeSchimbat - pozitiaDorita)/100;
            frontRightViteza = putere + (valoareDeSchimbat - pozitiaDorita)/100;
            backLeftViteza = putere + (valoareDeSchimbat - pozitiaDorita)/100;
            backRightViteza = putere - (valoareDeSchimbat - pozitiaDorita)/100;

            frontLeftViteza = Range.clip(frontLeftViteza, -1,1);
            frontRightViteza = Range.clip(frontRightViteza, -1,1);
            backLeftViteza = Range.clip(backLeftViteza, -1,1);
            backRightViteza = Range.clip(backRightViteza, -1,1);

            frontLeft.setPower(frontLeftViteza);
            frontRight.setPower(frontRightViteza);
            backLeft.setPower(backLeftViteza);
            backRight.setPower(backRightViteza);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /*
    void SpreLinie(){
        while (opModeIsActive() && (luminozitate() < 0.5)) {
            frontLeft.setPower(-0.2);
            frontRight.setPower(0.2);
            backLeft.setPower(0.2);
            backRight.setPower(-0.2);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    double luminozitate() {
        NormalizedRGBA culoare = senzorLinie.getNormalizedColors();
        telemetry.addData("Nivel lumina: ",  "%4.2f", culoare.alpha);
        telemetry.update();

        return culoare.alpha;
    }

     */

    void ridica(int unde){
        encoder(motorKatanaDreapta, unde, 2000);
        encoder(motorKatanaStanga, unde, 2000);
    }

    void encoder(DcMotorEx motor,int pozitie, int viteza){

        motor.setTargetPosition(pozitie);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(viteza);
    }

    void deschideGheara(){
        servoGhearaStanga.setPosition(0.5);
        servoGhearaDreapta.setPosition(0);
    }
    void inchideGheara(){
        servoGhearaStanga.setPosition(0);
        servoGhearaDreapta.setPosition(0.4);
    }

    void Turn(double targetAngle){

        TurnPIDController pid = new TurnPIDController(targetAngle, 0.1,0,0);

        while(opModeIsActive() && Math.abs(targetAngle - PozitiaActuala())>1){

            double motorPower = pid.update(PozitiaActuala());

            frontRight.setPower(-motorPower);
            frontLeft.setPower(motorPower);
            backRight.setPower(-motorPower);
            backLeft.setPower(motorPower);
        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }


    void TineDirectia(double targetAngle, double timp) {

        ElapsedTime ceas = new ElapsedTime();
        ceas.reset();

        TurnPIDController pid = new TurnPIDController(targetAngle, 0.1,0,0);

        while (opModeIsActive() && Math.abs(targetAngle - PozitiaActuala())>1 && (ceas.time() < timp)) {
            double motorPower = pid.update(PozitiaActuala());

            frontRight.setPower(-motorPower);
            frontLeft.setPower(motorPower);
            backRight.setPower(-motorPower);
            backLeft.setPower(motorPower);
        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public double PozitiaActuala(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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

        //senzorLinie = hardwareMap.get(NormalizedColorSensor.class, "senzorCuloare");
        sensorRange = hardwareMap.get(DistanceSensor.class, "dis");

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}
