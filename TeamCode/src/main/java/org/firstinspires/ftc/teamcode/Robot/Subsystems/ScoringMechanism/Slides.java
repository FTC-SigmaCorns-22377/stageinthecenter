package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Utils.ProfiledPID;

@Config
public class Slides extends Subsystem {

    public static final double IN_POSITION = 0;


    static final double PULLEY_CIRCUMFERENCE = 4.409;
    static final double counts_per_revolution = 145.090909;
    public static double Kp = 0.6;
    public static double Kd = 1.8 * Math.sqrt(Kp * 0.0015);
    public static double Ki = 0;
//    public static double Kd = 0.01;
    public static double max_accel = 250;
    public static double max_velocity = 250;

    public static double DISTANCE_FOR_CONE = 8; // 9 inches or less means we still have the cone

    protected double slideTargetPosition = 0;
    protected double Kg = 0; // 0.09499 TODO: TUNE
    DcMotorEx vertical1;
    DcMotorEx vertical2;
    PIDCoefficients coefficients = new PIDCoefficients(Kp, Ki, Kd);
    MotionConstraint upConstraint = new MotionConstraint(max_accel, max_accel, max_velocity);
    MotionConstraint downConstraint = new MotionConstraint(max_accel, max_accel, max_velocity);
    ProfiledPID controller = new ProfiledPID(upConstraint, downConstraint, coefficients);
    private VoltageSensor batteryVoltageSensor;
    protected double current = 0;
    protected double power = 0;

    private ElapsedTime timer;
    private double lastPosition;
    private double lastTime;

    private double velocityInchesPerSecond = 0;

    private SlideHeight slideHeight = SlideHeight.L0;

    public void commonInit(HardwareMap hwMap) {
        vertical1 = hwMap.get(DcMotorEx.class, "slides1");
        vertical2 = hwMap.get(DcMotorEx.class, "slides2");
        // TODO, set direction
        vertical1.setDirection(DcMotorSimple.Direction.REVERSE);
        vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        this.timer = new ElapsedTime();
        this.lastPosition = this.getSlidePosition();
        this.lastTime = this.timer.milliseconds();
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);
        vertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
        upConstraint = new MotionConstraint(max_accel, max_accel / 2, max_velocity);
        downConstraint = new MotionConstraint(max_accel, max_accel / 2, max_velocity);

        vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        updateTarget();

//        if (getSlidePosition() < 4 && currentLimitExceeded()) {
//            vertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            vertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////            vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }

        updatePID();
        getSlidePosition();
        this.updateVelocity();
        Dashboard.packet.put("Vertical PID Target Inches", this.getPIDTargetInches());
        Dashboard.packet.put("Vertical Position Inches", this.getSlidePosition());
        Dashboard.packet.put("Vertical Position Actual", vertical1.getCurrentPosition());
        Dashboard.packet.put("Vertical Deviation Inches", this.getPIDTargetDeviation());
        Dashboard.packet.put("DANGER", 0); // <- will be overridden by command
    }

    @Override
    public void shutdown() {
        vertical1.setPower(0);
        vertical2.setPower(0);
    }

    private void updateTarget() {
        switch (slideHeight) {
            case L0:
                slideTargetPosition = IN_POSITION;
                break;
            case L1:
                slideTargetPosition = 2;
                break;
            case L2:
                slideTargetPosition = 6;
                break;
            case L3:
                slideTargetPosition = 11;
                break;
            case L4:
                slideTargetPosition = 16;
                break;
            case L5:
                slideTargetPosition = 20;
                break;
        }

    }

    private void updateVelocity() {
        double newPosition = countsToInches(getSlidePosition());
        double newTime = timer.seconds();

        this.velocityInchesPerSecond = Math.abs(newPosition - this.lastPosition) / (newTime - this.lastTime);

        Dashboard.packet.put("Vertical Extension Velocity", this.velocityInchesPerSecond);
        this.lastPosition = newPosition;
        this.lastTime = newTime;
    }

    public double getVelocityInchesPerSecond() {
        return this.velocityInchesPerSecond;
    }
    public double getSpeedInchesPerSecond() {
        double speed = Math.abs(this.velocityInchesPerSecond);

        return Math.max(speed, 0.0001);
    }

    protected void updatePID() {
        double measuredPosition = getSlidePosition();
        double power = controller.calculate(slideTargetPosition, measuredPosition);
        if (slideTargetPosition > IN_POSITION * 2) {
            power += Kg;
        } else {
            power -= Kg;
        }

        if (currentLimitExceeded()) {
            power /= 4;
        }

        vertical1.setPower(-power);
        vertical2.setPower(-power);
        this.power = power;
        Dashboard.packet.put("measured slide position", measuredPosition);
        Dashboard.packet.put("target slide position", controller.getTargetPosition());
        Dashboard.packet.put("slide power", power);
        current = vertical1.getCurrent(AMPS);
        Dashboard.packet.put("vertical current amps",current);
    }

    /**
     * get position of the linear slides
     *
     * @return average encoder position of the slides
     */
    public double getSlidePosition() {
        return countsToInches((vertical1.getCurrentPosition() + vertical2.getCurrentPosition()) / 2.0);
//        return countsToInches((vertical1.getCurrentPosition()));
    }

    public double getSlideTargetPosition() {
        return slideTargetPosition;
    }
    public void setSlideHeight(SlideHeight slideHeight) { this.slideHeight = slideHeight; }

    public void setSlideTargetPosition(Double increment){
        this.slideTargetPosition = slideTargetPosition + increment;
    }

    public double getPIDTargetInches() {
        return controller.getTargetPosition();
    }

    public double getPIDTargetDeviation() {
        return Math.abs(this.getPIDTargetInches() - this.getSlidePosition());
    }

    public void updateTargetPosition(double targetpos) {
        this.slideTargetPosition = targetpos;
    }

    public boolean isMovementFinished() {
        return controller.isDone();
    }

    public double countsToInches(double counts) {
        return -(counts / counts_per_revolution) * PULLEY_CIRCUMFERENCE;
    }

    public boolean currentLimitExceeded() {
        return Math.abs(current) > 4;
    }

    public double getCurrent() {
        return current;
    }

    public double getPower() {
        return this.power;
    }

    public boolean slideIsDown() {
        return getSlidePosition() < 0.25;
    }


    public enum SlideHeight {
        L0,
        L1,
        L2,

        L3,
        L4,
        L5
    }

}
