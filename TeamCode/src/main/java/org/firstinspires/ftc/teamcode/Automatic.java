package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@Autonomous(name="Automatic", group="Linear Opmode")
@Disabled
public class Automatic extends LinearOpMode {

    /* Declare OpMode members. */
    private final ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV    = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION    = 20;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED             = 0.3;
    static final double TURN_SPEED              = 0.5;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    Servo carasouelServo;

    @Override public void runOpMode() {

        //Initialize the drive system variables.

        leftMotor = hardwareMap.dcMotor.get("Left_Motor"); // initializing left motor
        rightMotor = hardwareMap.dcMotor.get("Right_Motor"); // initializing right motor
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        armMotor = hardwareMap.dcMotor.get("Arm_Motor"); // initializing arm motor

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        carasouelServo = hardwareMap.servo.get("Carasouel_Servo");
        carasouelServo.setPosition(0.5);
        carasouelServo.setDirection(Servo.Direction.REVERSE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        
        //going to carasouel
        carasouelServo.setPosition(1.0);
        encoderDrive(DRIVE_SPEED,  -5,  -5, 0.7); // move back closer to carasouel
        encoderDrive(TURN_SPEED, 1.5,-1.5,0.5); // turn to the left
        //encoderDrive(0.3,-4.5, -4.5, 1.5); // move backwards
        encoderDrive(0.3,-1.5, -1.5, 0.5); // move backwards
        encoderDrive(0.2, -1, -1, 0.5);
        encoderDrive(0.1, -0.5,-0.5,0.5);
        sleep(3000);
        carasouelServo.setPosition(0.5);
        
        encoderDrive(0.3, 1.0, 1.0, 1.0); // moves forwards
        encoderDrive(0.3, -0.75, 0.75, 1.15); // moves right
        armMotor.setPower(-0.5);
        encoderDrive(1, 10, 10, 2.5); // moves forwards
        armMotor.setPower(0.0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftMotor.isBusy() && rightMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

