package org.firstinspires.ftc.teamcode.TeleOp.Decode2025;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//lists the code as a teleop
@TeleOp(name="DecodeTesting", group="Decode")

public class DecodeTesting extends LinearOpMode {

    // declare drivetrain motors
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor fl = null;
    private DcMotor fr = null;

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

    //declare other variables
    double LaunchPrevPos = 0;
    double LaunchVel = 0;
    String LaunchVelStr = "";

    //runtime Declare
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        // init and set up motors (gets config from driver hub, so code knows which port things are in)
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        launch = hardwareMap.get(DcMotorEx.class, "launcher");
        launch2 = hardwareMap.get(DcMotorEx.class,"launcher2");
        FrontIntake = hardwareMap.get(DcMotor.class,"FrontIntake");

        //init servos
        BackIntake = hardwareMap.get(CRServo.class, "BackIntake");

        //set zero power behavior (crazy)
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        launch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

        //believe it or not, this waits for start
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            //launcher velocity telemetry
            if (runtime.seconds() > 0.1) {
                LaunchVel = launch.getCurrentPosition() - LaunchPrevPos;
                LaunchVelStr = Double.toString(LaunchVel);
                telemetry.addData("Vel", LaunchVelStr);
                if (LaunchVel > 165){
                    telemetry.addLine("Ready to Fire!");
                }
                telemetry.update();
                LaunchPrevPos = launch.getCurrentPosition();
                runtime.reset();
            }

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
                launch.setVelocity(LauncherPower);
                launch2.setVelocity(LauncherPower);
            } else if (gamepad2.right_trigger > 0) {
                launch.setPower(-0.1);
                launch2.setPower(-0.1);
            } else {
                launch.setPower(0);
                launch2.setPower(0);
            }

            //intake controls
            //front
            if (gamepad2.dpad_up) {
                FrontIntake.setPower(IntakePower);
            } else if (gamepad2.dpad_down) {
                FrontIntake.setPower(-IntakePower);
            } else {
                FrontIntake.setPower(0);
            }
            //back
            if (gamepad2.a) {
                BackIntake.setPower(1);
            } else if (gamepad2.b) {
                BackIntake.setPower(-1);
            } else {
                BackIntake.setPower(0);
            }

        } //end bracket for loop, code after this won't run until the stop button is pressed (which breaks comp rules)
    }
}

