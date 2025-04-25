package org.firstinspires.ftc.teamcode.Auto_Period;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto_Test")
public class Auto_Test1 extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx LFmotor;
    DcMotorEx RFmotor;
    DcMotorEx LBmotor;
    DcMotorEx RBmotor;
    static final double     COUNTS_PER_MOTOR_REV = 537.7;
    static final double     WHEEL_DIAMETER_INCHES = 96.0/25.4;
    static final double     COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / WHEEL_DIAMETER_INCHES * 3.1415;
    static final double     DRIVE_SPEED = 0.3;
    static final double     TURN_SPEED = 0.2;

    double SPEED = 0.3;  // Speed for movements
    double ROTATE = 384.5;  // Encoder value per rotation (based on your setup)
    double newTarget;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        LFmotor = hardwareMap.get(DcMotorEx.class, "LFmotor");
        LFmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RFmotor = hardwareMap.get(DcMotorEx.class, "RFmotor");
        RFmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LBmotor = hardwareMap.get(DcMotorEx.class, "LBmotor");
        LBmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RBmotor = hardwareMap.get(DcMotorEx.class, "RBmotor");
        RBmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set motor directions
        LFmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RBmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status:", "Ready");
        telemetry.update();

        // Wait for the start command
        waitForStart();
        runtime.reset();
        strafe(900,"right");
        movement(1250,"down");
        strafe(1300,"left");
        movement(1250,"down");
        strafe(750,"left");
        movement(2250,"up");
        movement(2500,"down");
        rotate(880,"counter");
        movement(950,"down");
    }

    // Method for diagonal movement
    public void diagonals(double target, String direction) {
        newTarget = target;
        if (direction.equals("right")) {
            LFmotor.setTargetPosition((int) newTarget + LFmotor.getCurrentPosition());
            RBmotor.setTargetPosition((int) newTarget + RBmotor.getCurrentPosition());
            LFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LFmotor.setPower(SPEED);
            RBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RBmotor.setPower(SPEED);
        } else {
            LBmotor.setTargetPosition((int) newTarget + LBmotor.getCurrentPosition());
            RFmotor.setTargetPosition((int) newTarget + RFmotor.getCurrentPosition());
            LBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LBmotor.setPower(SPEED);
            RFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RFmotor.setPower(SPEED);
        }

        // Wait until motors are done
        while (opModeIsActive() && (LFmotor.isBusy() || LBmotor.isBusy() || RFmotor.isBusy() || RBmotor.isBusy())) {
            telemetry.addData("Diagonals", "Running to target: " + newTarget);
            telemetry.update();
        }

        stopAllMotors();
    }

    // Method for strafing (sideways movement)
    public void strafe(double target, String direction) {
        newTarget = target;
        if (direction.equals("right")) {
            LFmotor.setTargetPosition((int) newTarget + LFmotor.getCurrentPosition());
            LBmotor.setTargetPosition((int) -newTarget + LBmotor.getCurrentPosition());
            RFmotor.setTargetPosition((int) -newTarget + RFmotor.getCurrentPosition());
            RBmotor.setTargetPosition((int) newTarget + RBmotor.getCurrentPosition());
            LFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LFmotor.setPower(SPEED);
            LBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LBmotor.setPower(-SPEED);
            RFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RFmotor.setPower(-SPEED);
            RBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RBmotor.setPower(SPEED);
        } else {
            LFmotor.setTargetPosition((int) -newTarget + LFmotor.getCurrentPosition());
            LBmotor.setTargetPosition((int) newTarget + LBmotor.getCurrentPosition());
            RFmotor.setTargetPosition((int) newTarget + RFmotor.getCurrentPosition());
            RBmotor.setTargetPosition((int) -newTarget + RBmotor.getCurrentPosition());
            LFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LFmotor.setPower(-SPEED);
            LBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LBmotor.setPower(SPEED);
            RFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RFmotor.setPower(SPEED);
            RBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RBmotor.setPower(-SPEED);
        }

        // Wait until motors are done
        while (opModeIsActive() && (LFmotor.isBusy() || LBmotor.isBusy() || RFmotor.isBusy() || RBmotor.isBusy())) {
            telemetry.addData("Strafe", "Running to target: " + newTarget);
            telemetry.update();
        }

        stopAllMotors();
    }

    // Method for forward/backward movement
    public void movement(double target, String direction) {
        newTarget = target;
        if (direction.equals("up")) {
            LFmotor.setTargetPosition((int) newTarget + LFmotor.getCurrentPosition());
            LBmotor.setTargetPosition((int) newTarget + LBmotor.getCurrentPosition());
            RFmotor.setTargetPosition((int) newTarget + RFmotor.getCurrentPosition());
            RBmotor.setTargetPosition((int) newTarget + RBmotor.getCurrentPosition());
            LFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LFmotor.setPower(SPEED);
            LBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LBmotor.setPower(SPEED);
            RFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RFmotor.setPower(SPEED);
            RBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RBmotor.setPower(SPEED);
        } else {
            LFmotor.setTargetPosition((int) -newTarget + LFmotor.getCurrentPosition());
            LBmotor.setTargetPosition((int) -newTarget + LBmotor.getCurrentPosition());
            RFmotor.setTargetPosition((int) -newTarget + RFmotor.getCurrentPosition());
            RBmotor.setTargetPosition((int) -newTarget + RBmotor.getCurrentPosition());

            LFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LFmotor.setPower(-SPEED);
            LBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LBmotor.setPower(-SPEED);
            RFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RFmotor.setPower(-SPEED);
            RBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RBmotor.setPower(-SPEED);
        }

        // Wait until motors are done
        while (opModeIsActive() && (LFmotor.isBusy() || LBmotor.isBusy() || RFmotor.isBusy() || RBmotor.isBusy())) {
            telemetry.addData("F/B", "Running to target: " + newTarget);
            telemetry.update();
        }

        stopAllMotors();
    }

    // Method for rotating (turning) the robot
    // Method for rotating (turning) the robot
    public void rotate(double target, String direction) {
        // Calculate how many encoder ticks to move for the desired rotation
        newTarget = target;  // This assumes ROTATE is the encoder ticks per degree or unit of rotation
        // Set target positions for rotation (relative to current position)
        if (direction.equals("clockwise")) {
            LFmotor.setTargetPosition((int) newTarget + LFmotor.getCurrentPosition());
            LBmotor.setTargetPosition((int) newTarget + LBmotor.getCurrentPosition());
            RFmotor.setTargetPosition((int) -newTarget + RFmotor.getCurrentPosition());
            RBmotor.setTargetPosition((int) -newTarget + RBmotor.getCurrentPosition());
            LFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LFmotor.setPower(SPEED);
            LBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LBmotor.setPower(SPEED);
            RFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RFmotor.setPower(-SPEED);
            RBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RBmotor.setPower(-SPEED);
        } else { // Rotation to the left
            LFmotor.setTargetPosition((int) -newTarget + LFmotor.getCurrentPosition());
            LBmotor.setTargetPosition((int) -newTarget + LBmotor.getCurrentPosition());
            RFmotor.setTargetPosition((int) newTarget + RFmotor.getCurrentPosition());
            RBmotor.setTargetPosition((int) newTarget + RBmotor.getCurrentPosition());
            LFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LFmotor.setPower(-SPEED);
            LBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LBmotor.setPower(-SPEED);
            RFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RFmotor.setPower(SPEED);
            RBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RBmotor.setPower(SPEED);
        }
        // Wait until motors reach target positions
        while (opModeIsActive() && (LFmotor.isBusy() || LBmotor.isBusy() || RFmotor.isBusy() || RBmotor.isBusy())) {
            telemetry.addData("Rotate", "Running to target: " + newTarget);
            telemetry.update();
        }
        // Stop all motors after reaching target
        stopAllMotors();
    }


    // Stops all motors after movement is complete
    public void stopAllMotors() {
        LFmotor.setPower(0);
        LBmotor.setPower(0);
        RFmotor.setPower(0);
        RBmotor.setPower(0);
        LFmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LBmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Reset encoder mode to continue normal operations after stopping
        LFmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LBmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RFmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RBmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int newLFTarget;
        int newLBTarget;
        int newRFTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (!opModeIsActive()) return;

        // Determine new target position, and pass to motor controller
        newLFTarget = LFmotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newLBTarget = LBmotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRFTarget = RFmotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newRBTarget = RBmotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        LFmotor.setTargetPosition(newLFTarget);
        LBmotor.setTargetPosition(newLBTarget);
        RFmotor.setTargetPosition(newRFTarget);
        RBmotor.setTargetPosition(newRBTarget);

        // Turn On RUN_TO_POSITION
        LFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        LFmotor.setPower(Math.abs(speed));
        LBmotor.setPower(Math.abs(speed));
        RFmotor.setPower(Math.abs(speed));
        RBmotor.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() &&
                (LFmotor.isBusy() && RFmotor.isBusy())) {
            telemetry.addData("Path1",  "Running to %7d :%7d", newLFTarget,  newRFTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    LFmotor.getCurrentPosition(),
                    RFmotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        LFmotor.setPower(0);
        LBmotor.setPower(0);
        RFmotor.setPower(0);
        RBmotor.setPower(0);

        // Turn off RUN_TO_POSITION
        LFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RBmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(250);   // optional pause after each move
    }
}
