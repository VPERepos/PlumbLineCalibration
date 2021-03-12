package com.personal.PlumbLineCalibration;

//import com.panayotis.gnuplot.JavaPlot;
/*import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.stage.Stage;*/

import java.io.IOException;

public class App 
{
    
    public static void main( String[] args ) throws IOException
    {
        PlumbLineCalibrationAlgorithm CVProc = new PlumbLineCalibrationAlgorithm();

        
        CVProc.SetFocalLengthXPixel(6615);// Artificial image
        CVProc.SetFocalLengthYPixel(4678);
        CVProc.SetPixelWidthXmm(0.00155);//For example
        CVProc.SetPixelWidthXmm(0.00155);
        CVProc.SetBilateralFilterNeighbourHoodDiamter(9);
        CVProc.SetBilateralFilterSigmaColor(7);
        CVProc.SetBilateralSigmaSpace(7);
        CVProc.SetCannyApertureSize(3);
        CVProc.SetCannyThreshold1(20);
        CVProc.SetCannyThreshold2(50);
        CVProc.SetCannyUseL2Gradient(true);
        CVProc.SetDilateErodeKernelSize(5);
        CVProc.SetPlumbLineWidthLower(2);
        CVProc.SetPlumbLineWidthUpper(11);
        CVProc.SetSubpixelizationMethod("WeightedAverage"); // "BellCurveFit"
        CVProc.SetPathToImage("PlumbLineRasterDistorted.png");

        CVProc.ProcessImage();
        CVProc.GetLineMiddleCoordinates();
        CVProc.GroupPointsToLines();
        //CVProc.TransformLinesCoordinatesToCenter();
        CVProc.OptimizeCameraModel();
    }
}