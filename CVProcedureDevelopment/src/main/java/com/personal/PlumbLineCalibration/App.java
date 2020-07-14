package com.personal.PlumbLineCalibration;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;



import java.io.FileWriter;
import java.io.IOException;

import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.transform.DftNormalization;
import org.apache.commons.math3.transform.FastFourierTransformer;
import org.apache.commons.math3.transform.TransformType;
import org.apache.commons.math3.complex.*;
import org.apache.commons.math3.optim.*;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.PowellOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;

import com.panayotis.gnuplot.JavaPlot;
/*import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.stage.Stage;*/

import org.apache.commons.math3.fitting.*;
import org.apache.commons.math3.optim.nonlinear.vector.*;



public class App 
{
    
    public static void main( String[] args ) throws IOException
    {
        
        nu.pattern.OpenCV.loadShared();
        Mat ImgSource = Imgcodecs.imread("/home/vladi/Pictures/PlumbLineCalibration/ImageJExp/PlumbLineCalib1.jpg", Imgcodecs.IMREAD_GRAYSCALE);
        Mat Img = new Mat();
        Mat ImgWithVerticalLines = new Mat();
        ImgSource.copyTo(ImgWithVerticalLines);
        Core.bitwise_not( ImgSource, Img );

        Mat SmoothImg = new Mat();
        Imgproc.bilateralFilter(Img, SmoothImg, 9, 7, 7);
        Mat EdgesImg = new Mat();
        
               
        Imgproc.Canny(SmoothImg, EdgesImg, 20, 50, 3, true);
        
        int kernelSize=5;
        Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(2 * kernelSize + 1, 2 * kernelSize + 1),
        new Point(kernelSize, kernelSize));
        
        Imgproc.dilate(EdgesImg, EdgesImg, element);
        Imgproc.dilate(EdgesImg, EdgesImg, element);
        Imgproc.dilate(EdgesImg, EdgesImg, element);
        Imgproc.dilate(EdgesImg, EdgesImg, element);

        Imgproc.erode(EdgesImg, EdgesImg, element);
        Imgproc.erode(EdgesImg, EdgesImg, element);
        Imgproc.erode(EdgesImg, EdgesImg, element);
        Imgproc.erode(EdgesImg, EdgesImg, element);

        Mat ResizeImage = new Mat();
        Size ScaleSize = new Size(Img.cols()/2,Img.rows()/2);

        Imgproc.resize(EdgesImg, ResizeImage, ScaleSize , 0, 0, Imgproc.INTER_AREA);
        HighGui.namedWindow("Host image", HighGui.WINDOW_AUTOSIZE);
        HighGui.imshow("Host image",ResizeImage);
        HighGui.waitKey();
        ArrayList<LinkedList<Double>> VerticalLinesCoordinates = new ArrayList<LinkedList<Double>>(Img.rows());

        for (int i = 0; i < Img.rows(); i++)
        {
            ArrayList<Integer[]> IndexesOfROI = new ArrayList<Integer[]>(2);
            int pixelEdges;
            int pixelEdgesNext;
            
            ArrayList<Integer> ListOfTransitions = new ArrayList<Integer>();
            LinkedList<Double> ListOfLineXCoord = new LinkedList<Double>();
            
            for (int j = 0; j < EdgesImg.cols()-1; j++)
            {
                pixelEdges = (int)EdgesImg.get(i, j)[0];
                pixelEdgesNext = (int)EdgesImg.get(i, j+1)[0];
                
                if(pixelEdges < pixelEdgesNext)
                {
                    ListOfTransitions.add(new Integer(j));
                }
                else if(pixelEdges > pixelEdgesNext)
                {
                    ListOfTransitions.add(new Integer(j+1));
                }
                                
            }
            for(int Index = 0; Index < ListOfTransitions.size()-1; Index++)
            {
                if((ListOfTransitions.get(Index+1) - ListOfTransitions.get(Index))<= 11 && (ListOfTransitions.get(Index+1) - ListOfTransitions.get(Index)) >= 5 )
                {
                    ListOfLineXCoord.add(new Double(SubpixelPosition(i, (int)ListOfTransitions.get(Index), (int)ListOfTransitions.get(Index+1), SmoothImg, /*"WeightedAverage"*/"BellCurveFit")));
                }
            }
            VerticalLinesCoordinates.add(new LinkedList<Double>(ListOfLineXCoord));
            
            
            
        }
        
        ArrayList<LinkedList<Point>> VerticalLines = new ArrayList<LinkedList<Point>>();
        int LineCounter=0;
        double PrevDistance=0.0;
        double Distance = 0.0;      
        for(LinkedList<Double> Line : VerticalLinesCoordinates)
        {
            if(Line.size()>0)
            {
                Iterator<Double> LineIt = Line.iterator();
                if(Line.size()>1)
                {
                    Distance= 0.5*Math.abs(LineIt.next() - LineIt.next());
                    PrevDistance = Distance;
                }
                else
                {
                    Distance = PrevDistance;
                }
                 
                LineIt = Line.iterator();
                LineCounter++;
                while(LineIt.hasNext())
                {
                    LinkedList<Point> ActualLine = new LinkedList<Point>();
                    double ActXCoord = LineIt.next();
                    ActualLine.add(new Point(ActXCoord, LineCounter));
                    
                    for(int LocLineCounter=LineCounter; LocLineCounter<VerticalLinesCoordinates.size(); LocLineCounter++)
                    {
                        LinkedList<Double> LineToCheck = VerticalLinesCoordinates.get(LocLineCounter);
                        for(double LineToCheckElement : LineToCheck)
                        {
                            if(Math.abs(ActXCoord-LineToCheckElement)<=Distance)
                            {
                                ActualLine.add(new Point(LineToCheckElement, LocLineCounter));
                                LineToCheck.remove(LineToCheckElement);
                                break;
                            }
                        }
                    }
                    VerticalLines.add(new LinkedList<Point>(ActualLine));
                    if(Line.size()>0)
                    {   
                        Line.removeFirst();
                    }
                    LineIt = Line.iterator();
                }
            }
            
        }
        
        double Xh=0.0;
        double Yh=0.0;
        double K1=0.0;
        double K2=0.0;
        double P1=0.0;
        double P2=0.0;
        double K3=0.0;
        
        MaxEval maxEval = new MaxEval(50000);
        double[] initials = new double[]{
            Xh,
            Yh,
            K1,
            K2,
            P1,
            P2,
            K3,
        };  
        double[] result_point = new double[initials.length];
        InitialGuess start_values = new InitialGuess(initials);
        PowellOptimizer optimizer = new PowellOptimizer(1e-4, 1e-2);

        PlumbLineCalibrationAlgorithm K = new PlumbLineCalibrationAlgorithm();     
    
        /*ObjectiveFunction ob_negLnL = new ObjectiveFunction(ErrFunc  );
        try {
        PointValuePair result = optimizer.optimize(   , 1, start_values, maxEval);
        result_point = result.getPoint();
        }
        catch (TooManyEvaluationsException e) {
            for (int i = 0; i < result_point.length; i++) {
                result_point[i] = Double.NaN;
            }
        }*/
    }
    
    
    static double SubpixelPosition(int YCoord, int StartPos, int StopPos, Mat Img, String Method)
    {
        double SubPixelPos=0.0;
        
        if(Method == "BellCurveFit")
        {
            WeightedObservedPoints obs = new WeightedObservedPoints();
        
            for(int ImgIndex = 0; ImgIndex <= StopPos-StartPos; ImgIndex++)
            {
                obs.add(ImgIndex,  (double)Img.get(YCoord, ImgIndex+StartPos)[0]);
                
            }
                    
            double[] parameters = GaussianCurveFitter.create().fit(obs.toList());
                
            SubPixelPos = parameters[1]+StartPos;
        }
        else if(Method == "WeightedAverage")
        {
            double AccumulatedIntensity=0.0;
            double AccumulatedIntensityTimesPos=0.0;

            for(int ImgIndex = 0; ImgIndex <= StopPos-StartPos; ImgIndex++)
            {
                AccumulatedIntensityTimesPos+=ImgIndex*((double)Img.get(YCoord, ImgIndex+StartPos)[0]);
                AccumulatedIntensity+=((double)Img.get(YCoord, ImgIndex+StartPos)[0]);
            }
            SubPixelPos=StartPos+AccumulatedIntensityTimesPos/AccumulatedIntensity;
        }
        return SubPixelPos;
    }
}