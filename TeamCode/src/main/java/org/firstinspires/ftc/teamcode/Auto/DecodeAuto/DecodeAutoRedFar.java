package org.firstinspires.ftc.teamcode.Auto.DecodeAuto;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "DecodeAutoRedFar", preselectTeleOp = "DecodeTesting", group = "Decode")
public class DecodeAutoRedFar extends LinearOpMode {

    // declare drivetrain motors
    private DcMotorEx bl = null;
    private DcMotorEx br = null;
    private DcMotorEx fl = null;
    private DcMotorEx fr = null;

    //declare secondary motors
    private DcMotor launch = null;

    //declare servos
    private CRServo LeftIntake = null;
    private CRServo RightIntake = null;
    private CRServo Ejector = null;
    private Servo EUS = null; //Emergency Unsticking Service

    //declare sensors
    private RevColorSensorV3 Storage = null;

    // declare speed constants (immutable)
    final double diagonalStrafePower = 0.7; //diagonal strafe speed
    final double strafeScalar = 1.0; //strafing speed
    final double driveTrainScalar = 0.85; //overall movespeed
    final double LauncherPower = 0.75;
    final double IntakePower = 1;

    //declare position constants
    final double EUSActivePos = 0;
    final double EUSInactivePos = 1;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // init and set up motors (gets config from driver hub, so code knows which port things are in)
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        launch = hardwareMap.get(DcMotor.class, "launcher");

        //init servos
        LeftIntake = hardwareMap.get(CRServo.class, "LeftIntake");
        RightIntake = hardwareMap.get(CRServo.class, "RightIntake");
        Ejector = hardwareMap.get(CRServo.class, "Ejector");
        EUS = hardwareMap.get(Servo.class,"EUS");

        //init sensors
        Storage = hardwareMap.get(RevColorSensorV3.class, "StorageSensor");

        //set zero power behavior (crazy)
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // declare controller input variables (mutable)
        double leftPower;
        double rightPower;
        double leftStrafe;
        double rightStrafe;
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            resetMotorsAndTime();

            EUS.setPosition(EUSInactivePos);

            posForward(2000,3750);

            stop(0.1);

            posTurnRight(1000,575);

            stop(0.1);

            posForward(2000,200);

            stop(0.1);

            //posReverse(1000,1000);

            score();

            stop(0.1);

            posTurnRight(1000,500);
            posForward(2000,1000);

            stop(0.1);

            requestOpModeStop(); //stops opmode
        }
    }

    private void score() {
        launch.setPower(0.7);

        sleep(5000);

        Ejector.setPower(-1);
        EUS.setPosition(EUSActivePos);

        sleep(1000);

        launch.setPower(0);

        EUS.setPosition(EUSInactivePos);
        LeftIntake.setPower(1);
        RightIntake.setPower(-1);

        sleep(3000);

        launch.setPower(0);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
        Ejector.setPower(0);
        EUS.setPosition(EUSInactivePos);
    }

    private void posForward (double tps, int pos) {

        // tps = Ticks Per Second. (ie. position = 1000, tps = 500, will reach target position in 2s)
        bl.setTargetPosition(-pos);
        fl.setTargetPosition(-pos);
        br.setTargetPosition(pos);
        fr.setTargetPosition(pos);

        enablePos();

        bl.setVelocity(Math.abs(tps));
        br.setVelocity(Math.abs(tps));
        fl.setVelocity(Math.abs(tps));
        fr.setVelocity(Math.abs(tps));

        while (bl.isBusy() && fl.isBusy() && fr.isBusy() && br.isBusy() && opModeIsActive()){
            idle();
        }

        resetMotorsAndTime();
    }
    private void posTurnRight (double tps, int pos) {

        bl.setTargetPosition(-pos);
        fl.setTargetPosition(-pos);
        fr.setTargetPosition(-pos);
        br.setTargetPosition(-pos);

        enablePos();

        bl.setVelocity(Math.abs(tps));
        fl.setVelocity(Math.abs(tps));
        fr.setVelocity(Math.abs(tps));
        br.setVelocity(Math.abs(tps));

        while (bl.isBusy() && fl.isBusy() && fr.isBusy() && br.isBusy() && opModeIsActive()) {
            idle();
        }

        resetMotorsAndTime();
    }

    private void posTurnLeft (double tps, int pos) {

        bl.setTargetPosition(pos);
        fl.setTargetPosition(pos);
        fr.setTargetPosition(pos);
        br.setTargetPosition(pos);

        enablePos();

        bl.setVelocity(Math.abs(tps));
        fl.setVelocity(Math.abs(tps));
        fr.setVelocity(Math.abs(tps));
        br.setVelocity(Math.abs(tps));

        while (bl.isBusy() && fl.isBusy() && fr.isBusy() && br.isBusy() && opModeIsActive()) {
            idle();
        }

        resetMotorsAndTime();
    }

    private void posReverse (double tps, int pos) {

        bl.setTargetPosition(pos);
        fl.setTargetPosition(pos);
        fr.setTargetPosition(-pos);
        br.setTargetPosition(-pos);

        enablePos();

        bl.setVelocity(Math.abs(tps));
        fl.setVelocity(Math.abs(tps));
        fr.setVelocity(Math.abs(tps));
        br.setVelocity(Math.abs(tps));

        while (bl.isBusy() && fl.isBusy() && fr.isBusy() && br.isBusy() && opModeIsActive()){
            idle();
        }

        resetMotorsAndTime();
    }
    private void posStrafeLeft (double tps, int pos) {

        bl.setTargetPosition(pos);
        fl.setTargetPosition(-pos);
        fr.setTargetPosition(pos);
        br.setTargetPosition(-pos);

        enablePos();

        bl.setVelocity(Math.abs(tps));
        fl.setVelocity(Math.abs(tps));
        fr.setVelocity(Math.abs(tps));
        br.setVelocity(Math.abs(tps));

        while (bl.isBusy() && fl.isBusy() && fr.isBusy() && br.isBusy() && opModeIsActive()){
            idle();
        }

        resetMotorsAndTime();
    }

    private void posStrafeRight (double tps, int pos) {

        bl.setTargetPosition(-pos);
        fl.setTargetPosition(pos);
        fr.setTargetPosition(-pos);
        br.setTargetPosition(pos);

        enablePos();

        bl.setVelocity(Math.abs(tps));
        fl.setVelocity(Math.abs(tps));
        fr.setVelocity(Math.abs(tps));
        br.setVelocity(Math.abs(tps));

        while (bl.isBusy() && fl.isBusy() && fr.isBusy() && br.isBusy() && opModeIsActive()) {
            idle();
        }

        resetMotorsAndTime();


    }

    private void stop(double time) {

        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        while (opModeIsActive() && (runtime.seconds() <= Math.abs(time))) {
            idle();
        } runtime.reset();

    }

    public void trueStop (String mode) {
        switch (mode) {
            case "REVERSE":
                bl.setPower(-0.35);
                fl.setPower(-0.35);
                br.setPower(-0.35);
                fr.setPower(-0.35);

                while ((opModeIsActive() && (runtime.seconds() <= 0.2))) {
                    idle();
                }
                runtime.reset();
                break;

            case "LEFT":
                bl.setPower(0.25);
                fl.setPower(-0.25);
                fr.setPower(0.25);
                br.setPower(-0.25);
                while ((opModeIsActive() && (runtime.seconds() <= 0.15))) {
                    idle();
                }
                break;

            case "TURNRIGHT":
                bl.setPower(0.1);
                fl.setPower(0.1);
                fr.setPower(-0.1);
                br.setPower(-0.1);

                while ((opModeIsActive() && (runtime.seconds() <= 0.1))) {
                    idle();
                }
        }
    }

    private void resetMotorsAndTime() {

        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();
    }

    private void enablePos() {

        bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }
}
