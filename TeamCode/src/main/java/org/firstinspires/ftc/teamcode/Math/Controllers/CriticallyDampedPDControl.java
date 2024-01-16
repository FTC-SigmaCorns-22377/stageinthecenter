package org.firstinspires.ftc.teamcode.Math.Controllers;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * I have the best PID controllers, no one has PID controllers quite like I do.
 * <p>
 * My PID controllers effectively tune themselves.  They stop you from breaking your system.
 * <p>
 * My PID controllers are like the rust language - they just do everything the right way.
 */
public class CriticallyDampedPDControl implements FeedbackController {
	public boolean shouldFilter = false;
	protected double Kv;
	protected double Ka;
	protected double Kp;
	protected double Kd; // will be calculated from the other 3 values.
	protected double previous_error = 0;
	KalmanFilter filter = new KalmanFilter(0.3, 0.5, 5);
	ElapsedTime timer = new ElapsedTime();

	/**
	 * a controller that assuming Kv and Ka are correct, will yield a proportional derivative controller
	 * that is guaranteed to be critically damped no matter the value of Kp.
	 * Instead of Kp controlling overshoot, it's now effectively a speed term
	 * Kv and Ka are used to derive this value for Kd.
	 *
	 * @param Kp the proportional constant or more of a "speed" term
	 * @param Kv coefficient of velocity for model
	 * @param Ka coefficient of acceleration for model
	 */
	public CriticallyDampedPDControl(double Kp, double Kv, double Ka) throws Exception {
		this.Kp = Kp;
		this.Kv = Kv;
		this.Ka = Ka;
		this.Kd = solveKD(this.Kp, this.Kv, this.Ka);
	}

	/**
	 * given Kp, Ka, and Kv this will solve for the value of Kd that makes the position control loop
	 * critically damped
	 *
	 * @param Kp proportional term
	 * @param Kv velocity coefficient
	 * @param Ka acceleration coefficient
	 * @return Kd term that guarantees being critically damped.
	 */
	public static double solveKD(double Kp, double Kv, double Ka) {
		double sqrt_term = 2 * Math.sqrt(Ka * Kp);
		/* In our closed loop transfer function, Kp + Kd * s will be in the numerator of the transfer function.
		 * If Kd is negative which will result if we use too small of a value for Kp, our system will be non-minimum phase
		 * The act of being non-minimum phase places a zero in the left-half plane and while this does not make the system unstable,
		 * */

		if (sqrt_term < Kv) {
			return 0;
		}
		return sqrt_term - Kv;
	}

	@Override
	public double calculate(double reference, double state) {
		double dt = timer.seconds();
		timer.reset();
		double error = reference - state;
		double derivative = (error - previous_error) / dt;
		if (shouldFilter) {
			derivative = filter.estimate(derivative);
		}
		return error * Kp + derivative * Kd;
	}
}
