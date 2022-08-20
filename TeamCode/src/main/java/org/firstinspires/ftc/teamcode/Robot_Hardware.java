package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot_Hardware {
    // Motor stuff
    public DcMotor  backRight   = null;
    public DcMotor  backLeft    = null;
    public DcMotor  frontRight  = null;
    public DcMotor  frontLeft    = null;

    //Encoder stuff
    private double          COUNTS_PER_MOTOR_REV            = 1440 ;    // eg: TETRIX Motor Encoder
    private final double    DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double          WHEEL_DIAMETER_INCHES           = 4.0 ;     // For figuring circumference
    private double          COUNTS_PER_INCH                 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private double          COUNTS_PER_DEGREE               = COUNTS_PER_MOTOR_REV / 360;
    private double          DRIVE_SPEED                     = 0.6;
    private double          TURN_SPEED                      = 0.5;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Robot_Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        frontLeft = hwMap.get(DcMotor.class, "Front_Left");
        backLeft = hwMap.get(DcMotor.class, "Back_Right");
        frontRight = hwMap.get(DcMotor.class, "Front_Right");
        backRight = hwMap.get(DcMotor.class, "Back_Left");

        // Reverse motors
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // INTIIALIZE SERVOS HERE

        // CODE: https://github.com/AlessioToniolo/FTC-PID/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MaristBaseRobot2019_Quad.java
        //if (opModeIsActive()) {
//        if (true) {       // Swapped out to include in MaristBaseRobot
//
//            // Determine new target position, and pass to motor controller
//            newfrontLeftTarget = frontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
//            newfrontRightTarget = frontRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
//            newbackLeftTarget = backLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
//            newbackRightTarget = backRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
//
//            //
//            frontLeft.setTargetPosition(newfrontLeftTarget);
//            frontRight.setTargetPosition(newfrontRightTarget);
//            backLeft.setTargetPosition(newbackLeftTarget);
//            backRight.setTargetPosition(newbackRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            period.reset();
//            frontLeft.setPower(Math.abs(speed));
//            frontRight.setPower(Math.abs(speed));
//            backLeft.setPower(Math.abs(speed));
//            backRight.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while ((period.seconds() < timeoutS) &&
//                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() )) {
//                // Wait for Sequence to complete
//            }
//
//            // Stop all motion;
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }
//
//    public void pointTurnDegrees(double speed,
//                                 double deg,
//                                 double timeoutS) {
//
//        int newfrontLeftTarget;
//        int newfrontRightTarget;
//        int newbackLeftTarget;
//        int newbackRightTarget;
//
//        // Reverse inches
//        deg = deg * -1;
//
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Set to Limit of DRIVE_SPEED
//        if (Math.abs(speed) > DRIVE_SPEED) {
//            speed = DRIVE_SPEED; //
//        }
//
//        // Ensure that the opmode is still active
//        //if (opModeIsActive()) {
//        if (true) {       // Swapped out to include in MaristBaseRobot
//
//            // Determine new target position, and pass to motor controller
//            newfrontLeftTarget = frontLeft.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
//            newfrontRightTarget = frontRight.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
//            newbackLeftTarget = backLeft.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
//            newbackRightTarget = backRight.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
//
//            //
//            frontLeft.setTargetPosition(newfrontLeftTarget);
//            frontRight.setTargetPosition(newfrontRightTarget);
//            backLeft.setTargetPosition(newbackLeftTarget);
//            backRight.setTargetPosition(newbackRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            period.reset();
//            frontLeft.setPower(Math.abs(speed));
//            frontRight.setPower(Math.abs(speed));
//            backLeft.setPower(Math.abs(speed));
//            backRight.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while ((period.seconds() < timeoutS) &&
//                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() )) {
//                // Wait for Sequence to complete
//            }
//
//            // Stop all motion;
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }
    }
}
