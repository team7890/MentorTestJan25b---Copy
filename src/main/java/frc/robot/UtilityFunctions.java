// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorTuning;

/** Add your docs here. */
public class UtilityFunctions {
    public double joystickFix(double dInput) {
        double dOutput, m, b;
        dOutput = dInput;

        if (Math.abs(dInput) < OperatorTuning.dDriverDB) {
            dOutput = 0;
        }
        else{
            m = 1.0 / (1.0-OperatorTuning.dDriverDB);
            if (dInput > OperatorTuning.dDriverDB) {
                b = -m * OperatorTuning.dDriverDB;
                dOutput = m * dInput + b;
            }
            else {
                b = m * OperatorTuning.dDriverDB;
                dOutput = m * dInput + b;
            }
        }

        return dOutput;
    } 

    public static double limitVariable(double dMinValue, double dVariable, double dMaxValue) {
        double dValue;
        dValue = Math.max(dVariable, dMinValue);
        dValue = Math.min(dValue, dMaxValue);        
        return dValue;
    }
}
