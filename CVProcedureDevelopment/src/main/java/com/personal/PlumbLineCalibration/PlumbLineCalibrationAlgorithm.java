package com.personal.PlumbLineCalibration;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.calib3d.*;

import java.io.FileWriter;
import java.io.IOException;

import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.transform.DftNormalization;
import org.apache.commons.math3.transform.FastFourierTransformer;
import org.apache.commons.math3.transform.TransformType;
import org.apache.commons.math3.complex.*;
import org.apache.commons.math3.exception.TooManyEvaluationsException;
import org.apache.commons.math3.optim.*;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optimization.*;

import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.PowellOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;

import com.panayotis.gnuplot.JavaPlot;


import org.apache.commons.math3.fitting.*;
import org.apache.commons.math3.optim.nonlinear.vector.*;
import org.apache.commons.math3.analysis.MultivariateFunction;

public class PlumbLineCalibrationAlgorithm
{
    private String m_PathToImage;
    private Mat m_ImgSource;
    private Mat m_Img;
    private Mat m_SmoothImg;
    private Mat m_EdgesImg;
    private Mat m_DistortionFreeImg;
    
    private double m_FocalLengthXPixel;
    private double m_FocalLengthYPixel;
    private double m_PixelWidthXmm;
    private double m_PixelWidthYmm; 
    
    private ArrayList<LinkedList<Double>> m_VerticalLinesCoordinates;
    private ArrayList<LinkedList<Double>> m_HorizontalLinesCoordinates;

    ArrayList<LinkedList<Point>> m_VerticalLines;
    ArrayList<LinkedList<Point>> m_HorizontalLines;

    private int m_BilateralFilterNeighbourHoodDiamter;// default 9
    private int m_BilateralFilterSigmaColor;// default 7
    private int m_BilateralFilterSigmaSpace;// default 7
    private double m_CannyThreshold1;// default 20
    private double m_CannyThreshold2;// default 50
    private int m_CannyApertureSize;// default 3
    private boolean m_CannyUseL2Gradient;// default true
    private int m_DilateErodeKernelSize;// default 5
    private int m_PlumbLineWidthLower;// default 5
    private int m_PlumbLineWidthUpper;// default 11
    private String m_SubpixelizationMethod;// "WeightedAverage" or "BellCurveFit"

    public PlumbLineCalibrationAlgorithm()
    {
        nu.pattern.OpenCV.loadShared();
        m_PathToImage = new String();
        m_ImgSource = new Mat();
        m_Img = new Mat();
        m_SmoothImg = new Mat();
        m_EdgesImg = new Mat();
        m_DistortionFreeImg = new Mat();
        m_VerticalLinesCoordinates = new ArrayList<LinkedList<Double>>();
        m_HorizontalLinesCoordinates = new ArrayList<LinkedList<Double>>();
        m_VerticalLines = new ArrayList<LinkedList<Point>>();
        m_HorizontalLines = new ArrayList<LinkedList<Point>>();
        
    }
    public void SetPathToImage(String PathToImage)
    {
        m_PathToImage = PathToImage;
    }
    public void SetBilateralFilterNeighbourHoodDiamter(int BilateralFilterNeighbourHoodDiamter)
    {
        m_BilateralFilterNeighbourHoodDiamter = BilateralFilterNeighbourHoodDiamter;
    }
    public void SetBilateralFilterSigmaColor(int BilateralFilterSigmaColor)
    {
        m_BilateralFilterSigmaColor = BilateralFilterSigmaColor;
    }
    public void SetBilateralSigmaSpace(int BilateralFilterSigmaSpace)
    {
        m_BilateralFilterSigmaSpace = BilateralFilterSigmaSpace;
    }
    public void SetCannyThreshold1( double CannyThreshold1)
    {
        m_CannyThreshold1 = CannyThreshold1;
    }
    public void SetCannyThreshold2( double CannyThreshold2)
    {
        m_CannyThreshold2 = CannyThreshold2;
    }
    public void SetCannyApertureSize(int CannyApertureSize)
    {
        m_CannyApertureSize = CannyApertureSize;
    }
    public void SetCannyUseL2Gradient(boolean CannyUseL2Gradient)
    {
        m_CannyUseL2Gradient = CannyUseL2Gradient;
    }
    public void SetDilateErodeKernelSize(int DilateErodeKernelSize)
    {
        m_DilateErodeKernelSize = DilateErodeKernelSize;
    }
    public void SetPlumbLineWidthLower(int PlumbLineWidthLower)
    {
        m_PlumbLineWidthLower = PlumbLineWidthLower;
    }
    public void SetPlumbLineWidthUpper(int PlumbLineWidthUpper)
    {
        m_PlumbLineWidthUpper = PlumbLineWidthUpper;
    }
    public void SetSubpixelizationMethod(String SubpixelizationMethod)
    {
        m_SubpixelizationMethod = SubpixelizationMethod;
    }
    public void SetFocalLengthXPixel(double FocalLengthXPixel)
    {
        m_FocalLengthXPixel = FocalLengthXPixel;
    }
    public void SetFocalLengthYPixel(double FocalLengthYPixel)
    {
        m_FocalLengthYPixel = FocalLengthYPixel;
    }
    public void SetPixelWidthXmm(double PixelWidthXmm)
    {
        m_PixelWidthXmm = PixelWidthXmm;
    }
    public void SetPixelWidthYmm(double PixelWidthYmm)
    {
        m_PixelWidthYmm = PixelWidthYmm;
    }

    public void ClearAll()
    {
        m_PathToImage="";
        
        m_VerticalLinesCoordinates.clear();
        m_HorizontalLinesCoordinates.clear();
        m_VerticalLines.clear();
        m_HorizontalLines.clear();
    }

    public void ProcessImage()
    {
        
        m_ImgSource = Imgcodecs.imread(m_PathToImage, Imgcodecs.IMREAD_GRAYSCALE);

        Core.bitwise_not( m_ImgSource, m_Img );
        
        Imgproc.bilateralFilter(m_Img, m_SmoothImg, m_BilateralFilterNeighbourHoodDiamter, m_BilateralFilterSigmaColor, m_BilateralFilterSigmaSpace);
                
        Imgproc.Canny(m_SmoothImg, m_EdgesImg, m_CannyThreshold1, m_CannyThreshold2, m_CannyApertureSize, m_CannyUseL2Gradient);
        
        Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(2 * m_DilateErodeKernelSize + 1, 2 * m_DilateErodeKernelSize + 1),
        new Point(m_DilateErodeKernelSize, m_DilateErodeKernelSize));
        
        Imgproc.dilate(m_EdgesImg, m_EdgesImg, element);
        Imgproc.dilate(m_EdgesImg, m_EdgesImg, element);
        Imgproc.dilate(m_EdgesImg, m_EdgesImg, element);
        Imgproc.dilate(m_EdgesImg, m_EdgesImg, element);

        Imgproc.erode(m_EdgesImg, m_EdgesImg, element);
        Imgproc.erode(m_EdgesImg, m_EdgesImg, element);
        Imgproc.erode(m_EdgesImg, m_EdgesImg, element);
        Imgproc.erode(m_EdgesImg, m_EdgesImg, element);
    }

    private double SubpixelPositionX(int YCoord, int StartPos, int StopPos, Mat Img, String Method)
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

    private double SubpixelPositionY(int XCoord, int StartPos, int StopPos, Mat Img, String Method)
    {
        double SubPixelPos=0.0;
        
        if(Method == "BellCurveFit")
        {
            WeightedObservedPoints obs = new WeightedObservedPoints();
        
            for(int ImgIndex = 0; ImgIndex <= StopPos-StartPos; ImgIndex++)
            {
                obs.add(ImgIndex,  (double)Img.get(ImgIndex+StartPos, XCoord)[0]);
                
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
                AccumulatedIntensityTimesPos+=ImgIndex*((double)Img.get(ImgIndex+StartPos, XCoord)[0]);

                AccumulatedIntensity+=((double)Img.get(ImgIndex+StartPos, XCoord )[0]);
            }
            SubPixelPos=StartPos+AccumulatedIntensityTimesPos/AccumulatedIntensity;
        }
        return SubPixelPos;
    }

    public void GetLineMiddleCoordinates()
    {
        for (int i = 0; i < m_EdgesImg.rows(); i++)
        {
            int pixelEdges;
            int pixelEdgesNext;
            
            ArrayList<Integer> ListOfTransitions = new ArrayList<Integer>();
            LinkedList<Double> ListOfLineXCoord = new LinkedList<Double>();
            
            for (int j = 0; j < m_EdgesImg.cols()-1; j++)
            {
                pixelEdges = (int)m_EdgesImg.get(i, j)[0];
                pixelEdgesNext = (int)m_EdgesImg.get(i, j+1)[0];
                
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
                if((ListOfTransitions.get(Index+1) - ListOfTransitions.get(Index))<= m_PlumbLineWidthUpper && (ListOfTransitions.get(Index+1) - ListOfTransitions.get(Index)) >= m_PlumbLineWidthLower )
                {
                    ListOfLineXCoord.add(new Double(SubpixelPositionX(i, (int)ListOfTransitions.get(Index), (int)ListOfTransitions.get(Index+1), m_SmoothImg, m_SubpixelizationMethod)));
                }
            }
            m_VerticalLinesCoordinates.add(new LinkedList<Double>(ListOfLineXCoord));
             
        }

        for (int j = 0; j < m_Img.cols(); j++)
        {
            int pixelEdges;
            int pixelEdgesNext;
            
            ArrayList<Integer> ListOfTransitions = new ArrayList<Integer>();
            LinkedList<Double> ListOfLineYCoord = new LinkedList<Double>();
            
            for (int i = 0; i < m_EdgesImg.rows()-1; i++)
            {
                pixelEdges = (int)m_EdgesImg.get(i, j)[0];
                pixelEdgesNext = (int)m_EdgesImg.get(i+1, j)[0];
                
                if(pixelEdges < pixelEdgesNext)
                {
                    ListOfTransitions.add(new Integer(i));
                }
                else if(pixelEdges > pixelEdgesNext)
                {
                    ListOfTransitions.add(new Integer(i+1));
                }
                                
            }
            for(int Index = 0; Index < ListOfTransitions.size()-1; Index++)
            {
                if((ListOfTransitions.get(Index+1) - ListOfTransitions.get(Index))<= m_PlumbLineWidthUpper && (ListOfTransitions.get(Index+1) - ListOfTransitions.get(Index)) >= m_PlumbLineWidthLower )
                {
                    ListOfLineYCoord.add(new Double(SubpixelPositionY(j, (int)ListOfTransitions.get(Index), (int)ListOfTransitions.get(Index+1), m_SmoothImg, m_SubpixelizationMethod)));
                }
            }
            m_HorizontalLinesCoordinates.add(new LinkedList<Double>(ListOfLineYCoord));
             
        }
        
    }

    public void GroupPointsToLines()
    {
        int LineCounter=0;
        double PrevDistance=0.0;
        double Distance = 0.0;      
        for(LinkedList<Double> Line : m_VerticalLinesCoordinates)
        {
            if(Line.size()>0)
            {
                Iterator<Double> LineIt = Line.iterator();
                if(Line.size()>1)
                {
                    Distance= 0.9*Math.abs(LineIt.next() - LineIt.next());
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
                    
                    for(int LocLineCounter=LineCounter; LocLineCounter<m_VerticalLinesCoordinates.size(); LocLineCounter++)
                    {
                        LinkedList<Double> LineToCheck = m_VerticalLinesCoordinates.get(LocLineCounter);
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
                    m_VerticalLines.add(new LinkedList<Point>(ActualLine));
                    if(Line.size()>0)
                    {   
                        Line.removeFirst();
                    }
                    LineIt = Line.iterator();
                }
            }
            
        }

        LineCounter=0;
        PrevDistance=0.0;
        Distance = 0.0;      
        for(LinkedList<Double> Line : m_HorizontalLinesCoordinates)
        {
            if(Line.size()>0)
            {
                Iterator<Double> LineIt = Line.iterator();
                if(Line.size()>1)
                {
                    Distance= 0.9*Math.abs(LineIt.next() - LineIt.next());
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
                    double ActYCoord = LineIt.next();
                    ActualLine.add(new Point(LineCounter, ActYCoord));
                    
                    for(int LocLineCounter=LineCounter; LocLineCounter<m_HorizontalLinesCoordinates.size(); LocLineCounter++)
                    {
                        LinkedList<Double> LineToCheck = m_HorizontalLinesCoordinates.get(LocLineCounter);
                        for(double LineToCheckElement : LineToCheck)
                        {
                            if(Math.abs(ActYCoord-LineToCheckElement)<=Distance)
                            {
                                ActualLine.add(new Point(LocLineCounter, LineToCheckElement));
                                LineToCheck.remove(LineToCheckElement);
                                break;
                            }
                        }
                    }
                    m_HorizontalLines.add(new LinkedList<Point>(ActualLine));
                    if(Line.size()>0)
                    {   
                        Line.removeFirst();
                    }
                    LineIt = Line.iterator();
                }
            }
            
        }
        
    }
    public void TransformLinesCoordinatesToCenter()
    {
        for(LinkedList<Point>VerticalLine : m_VerticalLines)
        {
            for(Point ActualPoint : VerticalLine)
            {
                ActualPoint.x = ActualPoint.x - 0.5*m_ImgSource.rows();
                ActualPoint.y = ActualPoint.y - 0.5*m_ImgSource.cols();
            }

        }

        for(LinkedList<Point>HorizonatalLine : m_HorizontalLines)
        {
            for(Point ActualPoint : HorizonatalLine)
            {
                ActualPoint.x = ActualPoint.x - 0.5*m_ImgSource.rows();
                ActualPoint.y = ActualPoint.y - 0.5*m_ImgSource.cols();
            }

        }
    }
    
    public void OptimizeCameraModel()
    {
        double Xh=0.0;
        double Yh=0.0;
        double K1=1.0e-3;
        double K2=1.0e-4;
        double P1=1.0e-4;
        double P2=1.0e-4;
        double K3=1.0e-5;
        
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
        PowellOptimizer optimizer = new PowellOptimizer(1e-8, 1e-4);
        
        
        
        MultivariateFunction ErrFunc=new  MultivariateFunction() {
                //private static final long serialVersionUID = -8673650298627399464L;
                public double value(double[] Distortion) 
                {
                    double Error = 0.0;
                    Mat CameraMatrix = new Mat(3, 3, 5/*CV_64F*/);
                    Mat DistCoeffs = new Mat(5,1, 5/*CV_64F*/);

                    CameraMatrix.put(0, 0, m_FocalLengthXPixel); CameraMatrix.put(0, 1, 0.0); CameraMatrix.put(0, 2, Distortion[0]);
                    CameraMatrix.put(1, 0, 0.0); CameraMatrix.put(1, 1, m_FocalLengthYPixel); CameraMatrix.put(1, 2, Distortion[1]);
                    CameraMatrix.put(2, 0, 0.0); CameraMatrix.put(2, 1, 0.0); CameraMatrix.put(2, 2, 1.0);

                    DistCoeffs.put(0,0,Distortion[2]);
                    DistCoeffs.put(1,0,Distortion[3]);
                    DistCoeffs.put(2,0,Distortion[4]);
                    DistCoeffs.put(3,0,Distortion[5]);
                    DistCoeffs.put(4,0,Distortion[6]);
                    
                    for(LinkedList<Point>VerticalLine : m_VerticalLines)
                    {
                        if(VerticalLine.size()>=(2*m_ImgSource.cols()/3))
                        {
                            final WeightedObservedPoints obs = new WeightedObservedPoints();
                            MatOfPoint2f SrcPoints = new MatOfPoint2f();
                            MatOfPoint2f DstPoints = new MatOfPoint2f();
                            SrcPoints.fromList(VerticalLine);
                            DstPoints.fromList(VerticalLine);
                            Calib3d.undistortPoints(SrcPoints, DstPoints, CameraMatrix, DistCoeffs);
                            for(int i=0; i < VerticalLine.size(); i++)
                            {
                                double[] DstPoint = DstPoints.get(i, 0);
                                DstPoint[0] = DstPoint[0]*m_FocalLengthXPixel;
                                DstPoint[1] = DstPoint[1]*m_FocalLengthYPixel;
                                obs.add(DstPoint[1], DstPoint[0]);
                            }                            
                            
                            final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(1);
                            final double[] coeff = fitter.fit(obs.toList());

                            for(int i=0; i < VerticalLine.size(); i++)
                            {
                                double[] DstPoint = DstPoints.get(i, 0);
                                DstPoint[0] = DstPoint[0]*m_FocalLengthXPixel;
                                DstPoint[1] = DstPoint[1]*m_FocalLengthYPixel;
                                double diff = coeff[0]+coeff[1]*DstPoint[1]-DstPoint[0];
                                Error += diff*diff;
                            }

                            
                        }
                        
                    }
                                       
                    return Error;
                }
            };

        
        ObjectiveFunction OFErrFunc = new ObjectiveFunction(ErrFunc);
        try {
            PointValuePair result = optimizer.optimize(  OFErrFunc,start_values, maxEval);
            result_point = result.getPoint();
        }
        catch (TooManyEvaluationsException e) {
            for (int i = 0; i < result_point.length; i++) {
                result_point[i] = Double.NaN;
            }
        }
    } 
};