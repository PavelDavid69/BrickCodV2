package org.firstinspires.ftc.teamcode.PID_classes;

public class PDFController  {

    private double p, d, f;
    private final double lastError = 0.0;

    public PDFController(double p, double d, double f) {
        setCoefficients(p, d, f);
    }

    public void setCoefficients(double p, double d, double f) {
        this.p = p;
        this.d = d;
        this.f = f;
    }

    public double update(double target, double current) {
        double error = target - current;
        return p * error + d * (error - lastError) + f;
    }

//	public int update(int target, int current) {
//		int error = target - current;
//		return (int) (p * error + d * (error - lastError) + f);
//	}

}