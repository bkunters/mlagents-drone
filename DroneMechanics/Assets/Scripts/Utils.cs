using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public static class Utils
{
    public static float Sigmoid(float x)
    {
        return 1f / (1f + Mathf.Exp(-x));
    }

    /// <summary>
    /// approximates the dirac function
    /// </summary>
    /// <param name="x">Input</param>
    /// <param name="coefficient"></param>
    /// <returns>e^(-c|x|)</returns>
    public static float AlmostDirac(float x, float coefficient = 1f)
    {
        if (coefficient <= 0) throw new Exception("Choose a coefficient greater than 0.0");
        return Mathf.Exp(-coefficient * Mathf.Abs(x));
    }

    public static float Tanh(float x)
    {
        return (Mathf.Exp(x) - Mathf.Exp(-x)) / (Mathf.Exp(x) + Mathf.Exp(-x));
    }

    public static float Gaussian(float x)
    {
        return Mathf.Exp(-(x*x) / 2f);
    }

    public static float GaussianAbs(float x)
    {
        return Mathf.Exp(-Mathf.Abs(x) / 2f);
    }
}
