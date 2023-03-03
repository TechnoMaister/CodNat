package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FieldCentric")
@Disabled
public class FieldCentric extends LinearOpMode {

    private DcMotorEx motorKatanaDreapta = null;
    private DcMotorEx motorKatanaStanga = null;
    private DcMotor motorColector = null;

    private Servo servoGhearaStanga = null;
    private Servo servoGhearaDreapta = null;

    private DistanceSensor sensorRange;
    int pozitie;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_435)
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        GamepadEx driverOp = new GamepadEx(gamepad1);

        motorKatanaDreapta  = hardwareMap.get(DcMotorEx.class, "motorKatanaDreapta");
        motorKatanaStanga  = hardwareMap.get(DcMotorEx.class, "motorKatanaStanga");
        motorColector  = hardwareMap.get(DcMotorEx.class, "colector");

        servoGhearaStanga = hardwareMap.get(Servo.class, "ghearaStanga");
        servoGhearaDreapta = hardwareMap.get(Servo.class, "ghearaDreapta");

        waitForStart();

        while (!isStopRequested()) {

                drive.driveFieldCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX(),
                        imu.getRotation2d().getDegrees(),
                        false
                );
            }
        }
    }