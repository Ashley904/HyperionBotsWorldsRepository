package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name = "Limelight Goal Tracking Tuning", group = "Vision")
public class LimeLightGoalTrackingTuning extends LinearOpMode {

    // --- Gains (tunable via FTC Dashboard) ---
    public static double kP = 0.0179;
    public static double kI = 0.000;

    public static double maxPower = 0.85;
    public static double deadband = 1.5;  // degrees

    // Flip to -1.0 if the robot rotates the wrong way.
    public static double TURN_DIRECTION = 1.0;

    private Limelight3A limelight;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline (change if yours is elsewhere)

        // --- Drivetrain ---
        frontLeft  = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft   = hardwareMap.get(DcMotorEx.class, "BL");
        backRight  = hardwareMap.get(DcMotorEx.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Press PLAY to auto-aim at AprilTag.");
        telemetry.update();

        waitForStart();
        limelight.start();

        // --- PI controller state ---
        double error = 0;
        double integral = 0;
        double turnPower = 0;
        double lastTime = 0;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive()) {

            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            LLResult llResult = limelight.getLatestResult();

            // --- No-target safety ---
            if (llResult == null || !llResult.isValid()) {
                applyTurn(0);
                integral = 0; // clear windup while we have no target

                telemetry.addData("Has Target", false);
                telemetry.addData("Turn Power", 0.0);
                telemetry.update();
                continue;
            }

            // --- PI on tx ---
            error = llResult.getTx();
            integral += error * deltaTime;

            if (Math.abs(error) > deadband) {
                turnPower = (kP * error) + (kI * integral);
            } else {
                turnPower = 0;
                integral = 0; // reset integral when inside deadband
            }

            turnPower *= TURN_DIRECTION;
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

            applyTurn(turnPower);

            telemetry.addData("Has Target", true);
            telemetry.addData("tx (deg)",   "%.2f", error);
            telemetry.addData("ty (deg)",   "%.2f", llResult.getTy());
            telemetry.addData("ta (%)",     "%.2f", llResult.getTa());
            telemetry.addData("Integral",   "%.3f", integral);
            telemetry.addData("Turn Power", "%.3f", turnPower);
            telemetry.update();
        }

        applyTurn(0);
        limelight.stop();
    }

    /**
     * Mecanum rotate-in-place. Positive power = clockwise from above.
     */
    private void applyTurn(double turnPower) {
        frontLeft.setPower(turnPower);
        backLeft.setPower(turnPower);
        frontRight.setPower(-turnPower);
        backRight.setPower(-turnPower);
    }
}