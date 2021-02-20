package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class LogisticFunction {
    private double k;
    public LogisticFunction(double k)
    {
        this.k=k;
    }
    public double getPowerAt(double input, double a, double p, String s)
    {
        if(s.equals("drive")) {
            if (a >= 20) {
                return (((1 / (1 + Math.pow(Math.E, (-2 * k * input)))) + (.55 / (1 + Math.pow(Math.E, k * (input - (a - 6))))) - 0.85) * p / 0.7);
            } else {
                return (((1 / (1 + Math.pow(Math.E, (-4 * k * input)))) + (.55 / (1 + Math.pow(Math.E, k * (2 * input - (a))))) - 0.85) * p / 0.7);
            }
        }
        else if(s.equals("strafe")) {
            return ((((.5 / (1 + Math.pow(Math.E, (-4 * k * (input - 2))))) + (.5 / (1 + Math.pow(Math.E, 0.5 * k * ((2 * input) - (1.55 * a))))) - 1.1) * p / 0.7) + 1.143) * p;
        }
        return 0;
    }
}