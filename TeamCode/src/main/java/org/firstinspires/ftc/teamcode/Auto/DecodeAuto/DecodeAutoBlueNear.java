package org.firstinspires.ftc.teamcode.Auto.DecodeAuto;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "DecodeAutoBlueNear", preselectTeleOp = "DecodeTesting", group = "Decode")
public class DecodeAutoBlueNear extends LinearOpMode {

    // declare drivetrain motors
    private DcMotorEx bl = null;
    private DcMotorEx br = null;
    private DcMotorEx fl = null;
    private DcMotorEx fr = null;

    //declare secondary motors
    private DcMotorEx launch = null;
    private DcMotorEx launch2 = null;
    private DcMotor FrontIntake = null;

    //declare servos
    private CRServo BackIntake = null;

    // declare speed constants (immutable)
    final double diagonalStrafePower = 0.7; //diagonal strafe speed
    final double strafeScalar = 1.0; //strafing speed
    final double driveTrainScalar = 0.85; //overall movespeed
    final double LauncherPower = 1400;
    final double IntakePower = 0.33;

    //declare position constants
    final double EUSActivePos = -1;
    final double EUSInactivePos = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // init and set up motors (gets config from driver hub, so code knows which port things are in)
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        launch = hardwareMap.get(DcMotorEx.class, "launcher");
        launch2 = hardwareMap.get(DcMotorEx.class,"launcher2");
        FrontIntake = hardwareMap.get(DcMotor.class,"FrontIntake");

        //init servos
        BackIntake = hardwareMap.get(CRServo.class, "BackIntake");

        //set zero power behavior (crazy)
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.FLOAT));
        launch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // launcher setup
        launch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch.setVelocityPIDFCoefficients(300,0,0,17);
        launch.setDirection(DcMotorEx.Direction.REVERSE);
        launch2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch2.setVelocityPIDFCoefficients(300,0,0,17);
        launch2.setDirection(DcMotorEx.Direction.REVERSE);

        // other setup
        FrontIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // declare controller input variables (mutable)
        double leftPower;
        double rightPower;
        double leftStrafe;
        double rightStrafe;
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            resetMotorsAndTime();

            score2();

            stop(0.1);

            posStrafeLeft(2000,1000);

            posTurnLeft(2000,500);

            stop(0.1);

            posForward(2000,1750);

            requestOpModeStop(); //stops opmode
        }
    }

    /* private void score() {
        launch.setPower(0.625);

        sleep(6000);

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
    } */

    private void score2() {

        launch.setVelocity(1425);
        launch2.setVelocity(1425);

        sleep(4000);

        BackIntake.setPower(1);

        sleep(4000);

        FrontIntake.setPower(IntakePower);

        sleep(3000);

        FrontIntake.setPower(0);
        BackIntake.setPower(0);
        launch.setPower(0);
        launch2.setPower(0);
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

        bl.setTargetPosition(-pos);
        fl.setTargetPosition(pos);
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

        bl.setTargetPosition(pos);
        fl.setTargetPosition(-pos);
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
