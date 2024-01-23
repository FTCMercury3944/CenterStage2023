package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Auton")
public class AutonomousMode extends LinearOpMode
{
    private final ElapsedTime RUNTIME = new ElapsedTime();

    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.5;

    private double globalAngle;
    private double lastAngle;

    @Override
    public void runOpMode()
    {
        Rev2mDistanceSensor distLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distLeft");
        Rev2mDistanceSensor distRight = hardwareMap.get(Rev2mDistanceSensor.class, "distRight");

        waitForStart();

        driveForward(14.5); // Move to center position

        if (distLeft.getDistance(DistanceUnit.CM) < 10.0)
        {
            telemetry.addData("Detection", "LEFT");
            telemetry.update();

            gyroTurn(-87.0); // Turn 90 degrees left
            outtakeSlow();

            driveSideways(9.0); // Go to the next tile
            driveForward(-49.0); // Park

        }
        else if (distRight.getDistance(DistanceUnit.CM) < 10.0)
        {
            telemetry.addData("Detection", "RIGHT");
            telemetry.update();

            gyroTurn(87.0); // Turn 90 degrees right
            outtakeSlow();

            driveSideways(-9.0); // Go to the next tile
            driveForward(49.0); // Park
        }
        else
        {
            telemetry.addData("Detection", "MIDDLE");
            telemetry.update();

            driveForward(-1.5);
            outtakeSlow();

            driveSideways(-8.0); // Go to the side
            driveForward(11.0); // Go to the next tile
            driveSideways(57.0); // Park
        }
    }

    /**
     *  Calls encoderDrive() to move forward a specified distance.
     *
     *  @param distance The distance, in inches, a robot should travel forward;
     *                  positive is forward
     */
    public void driveForward(double distance)
    {
        encoderDrive(distance, false);
    }

    /**
     *  Calls encoderDrive() to move sideways a specified distance.
     *
     *  @param distance The distance, in inches, a robot should travel sideways;
     *                  positive is right
     */
    public void driveSideways(double distance)
    {
        encoderDrive(distance, true);
    }

    /**
     *  Calls intakeMove() to open the intake.
     */
    public void intake()
    {
        intakeMove(-1.0);
    }

    /**
     *  Calls intakeMove() to reverse the intake.
     */
    public void outtake()
    {
        intakeMove(0.5);
    }

    /**
     *  Calls intakeMove() to reverse the intake slowly.
     */
    public void outtakeSlow()
    {
        intakeMove(0.3);
    }

    /**
     *  Uses encoders to drive a specified distance in a specified direction at a specified speed.
     *  @param distance The distance, in inches, a robot should travel sideways;
     *                  positive is right or forward (depending on sidewaysSwitch)
     *  @param sidewaysSwitch If true, move left/right; if false, more forward/backward
     */
    private void encoderDrive(double distance, boolean sidewaysSwitch)
    {
        // Calculate distance
        int counts = (int)(-distance * COUNTS_PER_INCH);

        // Determine directional variables
        int directionalCounts = counts;
        double directionalVoltage = Math.abs(DRIVE_SPEED);
        if (sidewaysSwitch)
        {
            directionalCounts = -counts;
            directionalVoltage = -DRIVE_SPEED;
        }

        // Initialize the DC motors for each wheel
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Reverse right side
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Resist external force to motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder values
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        int newLeftFrontTarget = leftFront.getCurrentPosition() + counts;
        int newRightFrontTarget = rightFront.getCurrentPosition() + directionalCounts;
        int newLeftRearTarget = leftRear.getCurrentPosition() + directionalCounts;
        int newRightRearTarget = rightRear.getCurrentPosition() + counts;

        telemetry.addData("leftFront Power", -DRIVE_SPEED);
        telemetry.addData("leftFront Target", newLeftFrontTarget);
        telemetry.addData("leftRear Power", directionalVoltage);
        telemetry.addData("leftRear Target", newLeftRearTarget);
        telemetry.addData("rightFront Power", directionalVoltage);
        telemetry.addData("rightFront Target", newRightFrontTarget);
        telemetry.addData("rightRear Power", -DRIVE_SPEED);
        telemetry.addData("rightRear Target", newRightFrontTarget);

        // Pass target positions to motor controllers
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);

        // Begin running routine
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion
        leftFront.setPower(-DRIVE_SPEED);
        rightFront.setPower(directionalVoltage);
        leftRear.setPower(directionalVoltage);
        rightRear.setPower(-DRIVE_SPEED);

        // Keep running while the routine are still active, the timeout has not
        // elapsed, and both motors are still running
        while (opModeIsActive() &&
              (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()))
        {}

        // Stop all motion
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

        // Stop running to position
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     *  Resets the registered angle of the IMU for rotation.
     */
    private void resetAngle()
    {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        lastAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        globalAngle = 0.0;
    }

    /**
     *  Gets the current angle as determined by the IMU.
     *
     *  @return The current angle of the robot
     */
    private double getAngle()
    {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double deltaAngle = angle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle -= deltaAngle;
        lastAngle = angle;

        return globalAngle;
    }

    /**
     *  Uses gyroscope to turn a specified angle at a specified speed.
     *
     *  @param angle The angle, in degrees, a robot should turn;
     *               positive is clockwise
     */
    private void gyroTurn(double angle)
    {
        // Initialize the DC motors for each wheel
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Initialize IMU for movement tracking
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            )
        ));

        // Reset yaw
        imu.resetYaw();

        // Reverse right side
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Resist external force to motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Restart IMU movement tracking
        resetAngle();

        // Calculate voltage for turn
        double voltL, voltR;
        if (angle > 0) // Right turn
        {
            voltL = -DRIVE_SPEED;
            voltR = DRIVE_SPEED;
        }
        else if (angle < 0) // Left turn
        {
            voltL = DRIVE_SPEED;
            voltR = -DRIVE_SPEED;
        }
        else return;

        // Set power to begin rotating
        leftFront.setPower(voltL);
        rightFront.setPower(voltR);
        leftRear.setPower(voltL);
        rightRear.setPower(voltR);

        // Keep running until angle
        if (angle > 0) // Right turn
        {
            // On a right turn, the robot must get off zero first
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() < angle) {}
        }
        else // Left turn
            while (opModeIsActive() && getAngle() > angle) {}

        // Stop all motion
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

        // Stop running to position
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for rotation to stop
        sleep(1000);

        // Reset angle tracking on new heading
        resetAngle();
    }

    /**
     *  Moves the intake to a specified target position and stalls for a short period of time.
     *
     *  @param targetPos A value between -1.0 and 1.0, inclusive,
     *                   that the intake should go to. -1.0 is in,
     *                   1.0 is out.
     */
    private void intakeMove(double targetPos)
    {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        RUNTIME.reset();
        while (opModeIsActive() && RUNTIME.milliseconds() < 1000)
            intake.setPower(targetPos);

        intake.setPower(0.0);
    }
}
