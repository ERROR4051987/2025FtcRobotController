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

@TeleOp (name="FlywheelPFTuner")
public class FlywheelPFTuner extends LinearOpMode {

    // init up motor to use (in this case launch) and its power
    public DcMotorEx motor = null;
    public DcMotorEx motor2 = null;
    final double LauncherPower = 1450;
    double TargetVelocity = 0;

    // init PF
    double F = 0;
    double P = 0;

    //current best tuning: P = 200, F = 14

    // init step size
    double[] StepSizes = {10, 1, 0.1, 0.01};
    int StepIndex = 1;

    public void runOpMode() throws InterruptedException {

        // motor set up
        motor = hardwareMap.get(DcMotorEx.class,"launcher");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor2 = hardwareMap.get(DcMotorEx.class,"launcher2");
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);

        // pid activate!
        PIDFCoefficients PIDFCoefficients = new PIDFCoefficients(P,0,0,F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);

        // cool user-friendly telemetry
        telemetry.addLine("Init Complete");
        telemetry.addLine("Welcome to PF Tuning!");
        telemetry.addLine("Left & Right to change F, Up & Down to change P");
        telemetry.addLine("Right or Up for increase, opposite for decrease");
        telemetry.addLine("B to change step size");
        telemetry.addLine("Left Trigger to run motor");

        // waits for start (crazy)
        waitForStart();
        while (opModeIsActive()) {

            // change step size
            if (gamepad1.bWasPressed()) {
                StepIndex = (StepIndex + 1) % StepSizes.length;
            }


            // F Tuning
            if (gamepad1.dpadLeftWasPressed()) {
                F -= StepSizes[StepIndex];
            }
            if (gamepad1.dpadRightWasPressed()) {
                F += StepSizes[StepIndex];
            }

            // P Tuning
            if (gamepad1.dpadDownWasPressed()) {
                P -= StepSizes[StepIndex];
            }
            if (gamepad1.dpadUpWasPressed()) {
                P += StepSizes[StepIndex];
            }

            // update to retuned pf
            PIDFCoefficients = new PIDFCoefficients(P,0,0,F);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);
            motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);

            // set velocity to use
            if (gamepad1.left_trigger > 0) {
                TargetVelocity = (LauncherPower);
            } else {
                TargetVelocity = 0;
            }

            motor.setVelocity(TargetVelocity);
            motor2.setVelocity(TargetVelocity);

            double CurrentVelocity = motor.getVelocity();
            double Error = TargetVelocity - CurrentVelocity;

            //TELEMETRY!!!!!!11!!!!!!!1!!
            telemetry.addData("Target Velocity",TargetVelocity);
            telemetry.addData("Current Velocity",CurrentVelocity);
            telemetry.addData("Error",Error);
            telemetry.addLine("------------------");
            telemetry.addData("F (L/R)",F);
            telemetry.addData("P (U/D)",P);
            telemetry.addData("Step Size",StepSizes[StepIndex]);

            telemetry.update();



        }
    }
}
