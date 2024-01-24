package org.firstinspires.ftc.teamcode.Utils;

public class KalmanFilter {
    private double Q = 0;   //model covariance
    private double R = 0;   //sensor covariance
    private double p = 0;   //initial covariance guess
    private double K = 0;   //initial Kalman gain guess
    private double u = 0;   //encoder estimate
    private double z = 0;   //aprilTag estimate
    private double x = 0;   //total estimate
    public KalmanFilter(double Q, double R, double K, double p, double x){
        this.Q = Q;
        this.R = R;
        this.K = K;
        this.p = p;
        this.x = x;
    }

    public double update(double u, double z){
        x = x + u;  //update position from encoders
        p = p + Q;  //update covariance
        K = p/(p + R);  //update K
        x = x + K*(z-x);    //update position from encoder with aprilTag
        p = (1-K) * p;  //update covariance
        return x;
    }


}
