package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="TeleOp")
public class TeleOpMode extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        // Send status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the DC motors for each wheel
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Initialize DC motors for intake and climber and servo for launcher
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor climber = hardwareMap.get(DcMotor.class, "climber");
        CRServo launcher = hardwareMap.get(CRServo.class, "launcher");

        // Reverse right side and arm motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoder values
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Tell motors to run from power level
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell wheel and climber motors to resist external force
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell intake motor to give in to external force
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");

        // Wait for driver to press play
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            Orientation o = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("first", o.firstAngle);
            telemetry.addData("second", o.secondAngle);
            telemetry.addData("third", o.thirdAngle);
            telemetry.update();

            // Movement - left stick
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

            // Rotation - right stick
            double rx = gamepad1.right_stick_x * 0.5;

            // Intake - LT is in, RT is out
            double lt = gamepad1.left_trigger;
            double rt = gamepad1.right_trigger;

            // Climber - D-pad up is up, D-pad down is down
            boolean dpad_up = gamepad1.dpad_up;
            boolean dpad_down = gamepad1.dpad_down;

            // Launcher - x button
            boolean xBut = gamepad1.x;

            // Apply mecanum formulae to wheels, with a common denominator so no
            // aliasing appears around nyquist or -nyquist
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftRearPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightRearPower = (y + x - rx) / denominator;

            // Intake controls; LT is in, RT is out
            double intakePower = (lt == 0.0) ? -rt : (lt * 0.8);

            // Climber controls; D-pad up is up, D-pad down is down, otherwise stay still
            double climberPower = dpad_up ? -1.0 : dpad_down ? 1.0 : 0.0;

            // Launcher controls; X launches
            double launcherPosition = xBut ? 1.0 : 0.0;

            // Send power to motors if not too close
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            // Send power to intake and climber
            intake.setPower(intakePower);
            climber.setPower(climberPower);

            // Send position to launcher
            launcher.setPower(launcherPosition);
        }
    }
}
