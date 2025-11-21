package org.firstinspires.ftc.teamcode.TeleOp.Decode2025;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//lists the code as a teleop
@TeleOp(name="DecodeStable", group="Decode")

public class DecodeStable extends LinearOpMode {

    // declare drivetrain motors
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor fl = null;
    private DcMotor fr = null;

    //declare secondary motors
    private DcMotor launch = null;

    //declare servos
    private CRServo LeftIntake = null;
    private CRServo RightIntake = null;
    private CRServo Ejector = null;
    private Servo EUS = null; //Emergency Unsticking Service

    // declare speed constants (immutable)
    final double diagonalStrafePower = 0.7; //diagonal strafe speed
    final double strafeScalar = 1.0; //strafing speed
    final double driveTrainScalar = 0.85; //overall movespeed
    final double LauncherPower = 0.7;
    final double IntakePower = 1;

    //declare position constants
    final double EUSActivePos = 0;
    final double EUSInactivePos = 1;

    public void runOpMode() throws InterruptedException {

        // init and set up motors (gets config from driver hub, so code knows which port things are in)
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        launch = hardwareMap.get(DcMotor.class, "launcher");

        //init servos
        LeftIntake = hardwareMap.get(CRServo.class, "LeftIntake");
        RightIntake = hardwareMap.get(CRServo.class, "RightIntake");
        Ejector = hardwareMap.get(CRServo.class, "Ejector");
        EUS = hardwareMap.get(Servo.class,"EUS");

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

        //believe it or not, this waits for start
        waitForStart();

        while (opModeIsActive()) {

            // get controller inputs for movement
            leftPower = gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
            leftStrafe = gamepad1.left_trigger;
            rightStrafe = gamepad1.right_trigger;

            //set drivetrain motor powers
            bl.setPower(leftPower * driveTrainScalar);
            fl.setPower(leftPower * driveTrainScalar);
            br.setPower(rightPower * driveTrainScalar);
            fr.setPower(rightPower * driveTrainScalar);

            // strafe left and right
            if (gamepad1.left_trigger > 0) {

                bl.setPower(-leftStrafe * strafeScalar);
                fl.setPower(leftStrafe * strafeScalar);
                br.setPower(-leftStrafe * strafeScalar);
                fr.setPower(leftStrafe * strafeScalar);

            } else if (gamepad1.right_trigger > 0) {

                bl.setPower(rightStrafe * strafeScalar);
                fl.setPower(-rightStrafe * strafeScalar);
                br.setPower(rightStrafe * strafeScalar);
                fr.setPower(-rightStrafe * strafeScalar);
            }

            //strafe diagonals
            if (gamepad1.dpad_up && gamepad1.left_bumper) {

                bl.setPower(-diagonalStrafePower);
                fl.setPower(0);

                br.setPower(0);
                fr.setPower(diagonalStrafePower);
                // upLeft

            } else if (gamepad1.dpad_down && gamepad1.left_bumper) {

                bl.setPower(0);
                fl.setPower(diagonalStrafePower);

                br.setPower(-diagonalStrafePower);
                fr.setPower(0);
                // downLeft

            } else if (gamepad1.dpad_up && gamepad1.right_bumper) {

                //skibidi
                bl.setPower(0);
                fl.setPower(-diagonalStrafePower);

                br.setPower(diagonalStrafePower);
                fr.setPower(0);
                // upRight

            } else if (gamepad1.dpad_down && gamepad1.right_bumper) {

                bl.setPower(diagonalStrafePower);
                fl.setPower(0);

                br.setPower(0);
                fr.setPower(-diagonalStrafePower);
                // downRight

            } //end of strafe code

            //launcher controls
            if (gamepad2.left_trigger > 0) {
                launch.setPower(LauncherPower);
            }else {
                launch.setPower(0);
            }

            //intake controls
            if (gamepad2.dpad_up) {
                LeftIntake.setPower(IntakePower);
                RightIntake.setPower(-IntakePower);
            } else if (gamepad2.dpad_down) {
                LeftIntake.setPower(-IntakePower);
                RightIntake.setPower(IntakePower);
            } else {
                LeftIntake.setPower(0);
                RightIntake.setPower(0);
            }

            //ejector controls
            if (gamepad2.a) {
                Ejector.setPower(-1);
            } else if (gamepad2.b) {
                Ejector.setPower(1);
            } else {
                Ejector.setPower(0);
            }

            //Emergency Unsticking Service
            if (gamepad2.right_trigger > 0) {
                EUS.setPosition(EUSActivePos);
            } else {
                EUS.setPosition(EUSInactivePos);
            }

        } //end bracket for loop, code after this won't run until the stop button is pressed (which breaks comp rules)
    }
}

/*
Robot Control Documentation:
    Gamepad 1:
        Left Stick - Left Wheels
        Right Stick - Right Wheels
        Left Trigger - Strafe Left
        Right Trigger - Strafe Right
        D-Pad Up/Down - Diagonal Strafe Controls
        Left/Right Bumper - Diagonal Strafe Controls

    Gamepad 2:
        Left Trigger - Rev up Launcher
        D-Pad Up/Down - Operate Intake Servos
        A/B - Ejector
        Left/Right Stick Buttons - Emergency Unsticking Service
 */