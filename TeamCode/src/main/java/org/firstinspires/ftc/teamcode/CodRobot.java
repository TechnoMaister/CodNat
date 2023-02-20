package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="CodRobot")
public class CodRobot extends OpMode {

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    private DcMotorEx motorKatanaDreapta = null;
    private DcMotorEx motorKatanaStanga = null;
    private DcMotor motorColector = null;

    private Servo servoGhearaStanga = null;
    private Servo servoGhearaDreapta = null;

    private DistanceSensor sensorRange;
    int pozitie;

    //NormalizedColorSensor senzorLinie;

    @Override
    public void init() {

        hardwareMap();

    }

    @Override
    public void loop() {

        miscare();
        executare();

    }


    void executare(){

        //if(gheara()==1){
        if (gamepad2.y)
            pozitie = 8500;
        else if (gamepad2.b)
            pozitie = 6000;
        else if (gamepad2.a)
            pozitie = 3750;
        else if (gamepad2.right_trigger != 0)
            pozitie = 300;
            //}
        else if(gamepad2.x) {
            pozitie = 0;
            deschidereGheara();
        }

        if(motorKatanaStanga.getCurrentPosition()<8000 && motorKatanaDreapta.getCurrentPosition()<8000 && gamepad2.right_trigger!=0)
            pozitie=pozitie+500;
        else if(gamepad2.left_trigger!=0 && motorKatanaStanga.getCurrentPosition()>3500 && motorKatanaDreapta.getCurrentPosition()>3500)
            pozitie=pozitie-500;

        encoder(motorKatanaDreapta, pozitie, 2000);
        encoder(motorKatanaStanga, pozitie, 2000);

        if(gamepad2.dpad_up)
            deschidereGheara();
        else if(gamepad2.dpad_down)
            inchidereGheara();


        if(gamepad2.right_bumper){
            motorColector.setPower(-1);
        }else{
            motorColector.setPower(0);
        }
        if(sensorRange.getDistance(DistanceUnit.CM)<10 && !motorKatanaDreapta.isBusy() && !motorKatanaStanga.isBusy() && motorKatanaDreapta.getCurrentPosition()<5 && motorKatanaStanga.getCurrentPosition()<5 && !
                gamepad2.dpad_up)
            inchidereGheara();

        telemetry.addData("TarghetKatana",pozitie);
        telemetry.addData("range", sensorRange.getDistance(DistanceUnit.CM));
        telemetry.addData("Pozitie dreapta",motorKatanaDreapta.getCurrentPosition());
        telemetry.addData("Pozitie stanga",motorKatanaStanga.getCurrentPosition());
        telemetry.update();
    }

    void encoder(DcMotorEx motor,int pozitie, int viteza){

        motor.setTargetPosition(pozitie);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(viteza);
    }

    void inchidereGheara(){
        servoGhearaStanga.setPosition(0);
        servoGhearaDreapta.setPosition(0.4);
    }
    void deschidereGheara(){
        servoGhearaStanga.setPosition(0.5);
        servoGhearaDreapta.setPosition(0);
    }
    int gheara(){
        if(servoGhearaDreapta.getPosition()!=0 && servoGhearaStanga.getPosition()==0.5)
            return 1;
        else
            return 0;
    }

    void miscare(){

        double fataSpate = -gamepad1.right_stick_y;
        double stangaDreapta = gamepad1.right_stick_x;
        double rotatie = -(gamepad1.right_trigger-gamepad1.left_trigger);
        double viteza = 1;
        if(gamepad1.left_bumper)
            viteza =2;

        double fl = Range.clip(fataSpate - stangaDreapta - rotatie,-1,1);
        double fr = Range.clip(fataSpate + stangaDreapta + rotatie,-1,1);
        double bl = Range.clip(fataSpate + stangaDreapta - rotatie,-1,1);
        double br = Range.clip(fataSpate - stangaDreapta + rotatie,-1,1);

        frontLeft.setPower(fr*fr*fr/viteza);
        frontRight.setPower(fl*fl*fl/viteza);
        backLeft.setPower(br*br*br/viteza);
        backRight.setPower(bl*bl*bl/viteza);
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

        sensorRange = hardwareMap.get(DistanceSensor.class, "dis");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;


    }
}