package com.personal.PlumbLineCalibration;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import javax.lang.model.util.ElementScanner6;

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
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.BOBYQAOptimizer;
import org.apache.commons.math3.optim.SimpleBounds;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;

import com.panayotis.gnuplot.GNUPlotParameters;
import com.panayotis.gnuplot.JavaPlot;
import com.panayotis.gnuplot.dataset.FileDataSet;
import com.panayotis.gnuplot.layout.StripeLayout;
import com.panayotis.gnuplot.plot.AbstractPlot;
import com.panayotis.gnuplot.plot.DataSetPlot;
import com.panayotis.gnuplot.style.NamedPlotColor;
import com.panayotis.gnuplot.style.PlotStyle;
import com.panayotis.gnuplot.style.Style;
import com.panayotis.gnuplot.swing.JPlot;
import com.panayotis.gnuplot.terminal.GNUPlotTerminal;
import com.panayotis.gnuplot.terminal.PostscriptTerminal;
import com.panayotis.gnuplot.utils.Debug;


import org.apache.commons.math3.fitting.*;
import org.apache.commons.math3.optim.nonlinear.vector.*;
import org.apache.commons.math3.analysis.MultivariateFunction;

public class PlumbLineCalibrationAlgorithm
{
    private String m_PathToImage;
    private Mat m_ImgSource;
    private final Mat m_Img;
    private final Mat m_SmoothImg;
    private final Mat m_EdgesImg;
    private final Mat m_DistortionFreeImg;
    
    private double m_FocalLengthXPixel;
    private double m_FocalLengthYPixel;
    private double m_PixelWidthXmm;
    private double m_PixelWidthYmm; 
    
    private final ArrayList<LinkedList<Double>> m_VerticalLinesCoordinates;
    

    ArrayList<LinkedList<Point>> m_VerticalLines;
    

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
        
        m_VerticalLines = new ArrayList<LinkedList<Point>>();
        
        
    }
    public void SetPathToImage(final String PathToImage)
    {
        m_PathToImage = PathToImage;
    }
    public void SetBilateralFilterNeighbourHoodDiamter(final int BilateralFilterNeighbourHoodDiamter)
    {
        m_BilateralFilterNeighbourHoodDiamter = BilateralFilterNeighbourHoodDiamter;
    }
    public void SetBilateralFilterSigmaColor(final int BilateralFilterSigmaColor)
    {
        m_BilateralFilterSigmaColor = BilateralFilterSigmaColor;
    }
    public void SetBilateralSigmaSpace(final int BilateralFilterSigmaSpace)
    {
        m_BilateralFilterSigmaSpace = BilateralFilterSigmaSpace;
    }
    public void SetCannyThreshold1( final double CannyThreshold1)
    {
        m_CannyThreshold1 = CannyThreshold1;
    }
    public void SetCannyThreshold2( final double CannyThreshold2)
    {
        m_CannyThreshold2 = CannyThreshold2;
    }
    public void SetCannyApertureSize(final int CannyApertureSize)
    {
        m_CannyApertureSize = CannyApertureSize;
    }
    public void SetCannyUseL2Gradient(final boolean CannyUseL2Gradient)
    {
        m_CannyUseL2Gradient = CannyUseL2Gradient;
    }
    public void SetDilateErodeKernelSize(final int DilateErodeKernelSize)
    {
        m_DilateErodeKernelSize = DilateErodeKernelSize;
    }
    public void SetPlumbLineWidthLower(final int PlumbLineWidthLower)
    {
        m_PlumbLineWidthLower = PlumbLineWidthLower;
    }
    public void SetPlumbLineWidthUpper(final int PlumbLineWidthUpper)
    {
        m_PlumbLineWidthUpper = PlumbLineWidthUpper;
    }
    public void SetSubpixelizationMethod(final String SubpixelizationMethod)
    {
        m_SubpixelizationMethod = SubpixelizationMethod;
    }
    public void SetFocalLengthXPixel(final double FocalLengthXPixel)
    {
        m_FocalLengthXPixel = FocalLengthXPixel;
    }
    public void SetFocalLengthYPixel(final double FocalLengthYPixel)
    {
        m_FocalLengthYPixel = FocalLengthYPixel;
    }
    public void SetPixelWidthXmm(final double PixelWidthXmm)
    {
        m_PixelWidthXmm = PixelWidthXmm;
    }
    public void SetPixelWidthYmm(final double PixelWidthYmm)
    {
        m_PixelWidthYmm = PixelWidthYmm;
    }

    public void ClearAll()
    {
        m_PathToImage="";
        
        m_VerticalLinesCoordinates.clear();
        
        m_VerticalLines.clear();
        
    }

    public void ProcessImage()
    {
        
        m_ImgSource = Imgcodecs.imread(m_PathToImage, Imgcodecs.IMREAD_GRAYSCALE);

        Core.bitwise_not( m_ImgSource, m_Img );
        
        Imgproc.bilateralFilter(m_Img, m_SmoothImg, m_BilateralFilterNeighbourHoodDiamter, m_BilateralFilterSigmaColor, m_BilateralFilterSigmaSpace);
                
        Imgproc.Canny(m_SmoothImg, m_EdgesImg, m_CannyThreshold1, m_CannyThreshold2, m_CannyApertureSize, m_CannyUseL2Gradient);
        
        final Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(2 * m_DilateErodeKernelSize + 1, 2 * m_DilateErodeKernelSize + 1),
        new Point(m_DilateErodeKernelSize, m_DilateErodeKernelSize));
        
        Imgproc.dilate(m_EdgesImg, m_EdgesImg, element);
        Imgproc.dilate(m_EdgesImg, m_EdgesImg, element);
        Imgproc.dilate(m_EdgesImg, m_EdgesImg, element);
        Imgproc.dilate(m_EdgesImg, m_EdgesImg, element);

        Imgproc.erode(m_EdgesImg, m_EdgesImg, element);
        Imgproc.erode(m_EdgesImg, m_EdgesImg, element);
        Imgproc.erode(m_EdgesImg, m_EdgesImg, element);
        Imgproc.erode(m_EdgesImg, m_EdgesImg, element);

        /*final Mat ResizeImage = new Mat();
        final Size ScaleSize = new Size(m_ImgSource.cols()/4, m_ImgSource.rows()/4);
        Imgproc.resize(m_EdgesImg, ResizeImage, ScaleSize,0,0,Imgproc.INTER_AREA);
        HighGui.namedWindow("Distortion free image", HighGui.WINDOW_AUTOSIZE);
        HighGui.imshow("Distortion free image", ResizeImage);
        HighGui.waitKey();*/
        //Imgcodecs.imwrite("ImageCannyMask.jpg", m_EdgesImg);
    }

    private double SubpixelPositionX(final int YCoord, final int StartPos, final int StopPos, final Mat Img, final String Method)
    {
        double SubPixelPos=0.0;
        
        if(Method == "BellCurveFit")
        {
            final WeightedObservedPoints obs = new WeightedObservedPoints();
        
            for(int ImgIndex = 0; ImgIndex <= StopPos-StartPos; ImgIndex++)
            {
                obs.add(ImgIndex,  (double)Img.get(YCoord, ImgIndex+StartPos)[0]);
                
            }
                    
            final double[] parameters = GaussianCurveFitter.create().fit(obs.toList());
                
            SubPixelPos = parameters[1]+StartPos;
        }
        else if(Method == "WeightedAverage")
        {
            double AccumulatedIntensity=0.0;
            double AccumulatedIntensityTimesPos=0.0;

            for(int ImgIndex = 1; ImgIndex <= StopPos-StartPos; ImgIndex++)
            {
                double ImgIntensity = (double)Img.get(YCoord, ImgIndex+StartPos-1)[0];
                AccumulatedIntensityTimesPos+=ImgIndex*ImgIntensity;
                AccumulatedIntensity+=ImgIntensity;
            }
            SubPixelPos=StartPos+AccumulatedIntensityTimesPos/AccumulatedIntensity;
        }
        return SubPixelPos;
    }

    private double SubpixelPositionY(final int XCoord, final int StartPos, final int StopPos, final Mat Img, final String Method)
    {
        double SubPixelPos=0.0;
        
        if(Method == "BellCurveFit")
        {
            final WeightedObservedPoints obs = new WeightedObservedPoints();
        
            for(int ImgIndex = 0; ImgIndex <= StopPos-StartPos; ImgIndex++)
            {
                obs.add(ImgIndex,  (double)Img.get(ImgIndex+StartPos, XCoord)[0]);
                
            }
                    
            final double[] parameters = GaussianCurveFitter.create().fit(obs.toList());
                
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
            double pixelEdges;
            double pixelEdgesNext;
            
            final ArrayList<Integer> ListOfTransitions = new ArrayList<Integer>();
            final LinkedList<Double> ListOfLineXCoord = new LinkedList<Double>();
            
            for (int j = 0; j < m_EdgesImg.cols()-1; j++)
            {
                pixelEdges = m_EdgesImg.get(i, j)[0];
                pixelEdgesNext = m_EdgesImg.get(i, j+1)[0];
                
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
                    ListOfLineXCoord.add(new Double(SubpixelPositionX(i, (int)ListOfTransitions.get(Index)-1, (int)ListOfTransitions.get(Index+1)+1, m_SmoothImg, m_SubpixelizationMethod)));
                    
                }
            }
            m_VerticalLinesCoordinates.add(new LinkedList<Double>(ListOfLineXCoord));
             
        }
        
        //System.out.println("yo");
    }

    public void GroupPointsToLines()
    {
        int LineCounter=0;
        double PrevDistance=0.0;
        double Distance = 0.0;      
        for(final LinkedList<Double> Line : m_VerticalLinesCoordinates)
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

                    final LinkedList<Point> ActualLine = new LinkedList<Point>();
                    final double ActXCoord = LineIt.next();
                    ActualLine.add(new Point(ActXCoord, LineCounter));
                    
                    for(int LocLineCounter=LineCounter; LocLineCounter<m_VerticalLinesCoordinates.size(); LocLineCounter++)
                    {
                        final LinkedList<Double> LineToCheck = m_VerticalLinesCoordinates.get(LocLineCounter);
                        for(final double LineToCheckElement : LineToCheck)
                        {
                            if(Math.abs(ActXCoord-LineToCheckElement)<=10/*Distance*/)
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
        
            
        
        
    }
    public void TransformLinesCoordinatesToCenter()
    {
        for(final LinkedList<Point>VerticalLine : m_VerticalLines)
        {
            for(final Point ActualPoint : VerticalLine)
            {
                ActualPoint.x = ActualPoint.x - 0.5*m_ImgSource.rows();
                ActualPoint.y = ActualPoint.y - 0.5*m_ImgSource.cols();
            }

        }
        
    }
    
    public void OptimizeCameraModel()
    {
            
        
        
        final MultivariateFunction ErrFunc=new  MultivariateFunction() {
                //private static final long serialVersionUID = -8673650298627399464L;
                public double value(final double[] Distortion) 
                {
                    double Error = 0.0;
                    final Mat CameraMatrix = new Mat(3, 3, 6/*CV_64F*/);
                    final Mat DistCoeffs = new Mat(1,5, 6/*CV_64F*/);
                    double HxLoc = 0.0;
                    double HyLoc = 0.0;
                    double K1Loc = 0.0;
                    double K2Loc = 0.0;
                    double P1Loc = 0.0;
                    double P2Loc = 0.0;
                    double K3Loc = 0.0;
                    double K4Loc = 0.0;
                    double K5Loc = 0.0;
                    double K6Loc = 0.0;
                    if(Distortion.length == 2)
                    {
                        HxLoc = Distortion[0];
                        HyLoc = Distortion[1];
                    }
                    else if(Distortion.length == 3)
                    {
                        HxLoc = Distortion[0];
                        HyLoc = Distortion[1];
                        K1Loc = Distortion[2];
                    }
                    else if(Distortion.length == 4)
                    {
                        HxLoc = Distortion[0];
                        HyLoc = Distortion[1];
                        K1Loc = Distortion[2];
                        K2Loc = Distortion[3];
                    }
                    else if(Distortion.length == 5)
                    {
                        HxLoc = Distortion[0];
                        HyLoc = Distortion[1];
                        K1Loc = Distortion[2];
                        K2Loc = Distortion[3];
                        P1Loc = Distortion[4];
                    }
                    else if(Distortion.length == 6)
                    {
                        HxLoc = Distortion[0];
                        HyLoc = Distortion[1];
                        K1Loc = Distortion[2];
                        K2Loc = Distortion[3];
                        P1Loc = Distortion[4];
                        P2Loc = Distortion[5];
                    }
                    else if(Distortion.length == 7)
                    {
                        HxLoc = Distortion[0];
                        HyLoc = Distortion[1];
                        K1Loc = Distortion[2];
                        K2Loc = Distortion[3];
                        P1Loc = Distortion[4];
                        P2Loc = Distortion[5];
                        K3Loc = Distortion[6];
                    }
                    else{
                        HxLoc = Distortion[0];
                        HyLoc = Distortion[1];
                        K1Loc = Distortion[2];
                        K2Loc = Distortion[3];
                        P1Loc = Distortion[4];
                        P2Loc = Distortion[5];
                        K3Loc = Distortion[6];
                        K4Loc = Distortion[7];
                        K5Loc = Distortion[8];
                        K6Loc = Distortion[9];
                    }
                    CameraMatrix.put(0, 0, m_FocalLengthXPixel); CameraMatrix.put(0, 1, 0.0); CameraMatrix.put(0, 2, HxLoc);
                    CameraMatrix.put(1, 0, 0.0); CameraMatrix.put(1, 1, m_FocalLengthYPixel); CameraMatrix.put(1, 2, HyLoc);
                    CameraMatrix.put(2, 0, 0.0); CameraMatrix.put(2, 1, 0.0); CameraMatrix.put(2, 2, 1.0);

                    DistCoeffs.put(0,0,K1Loc);
                    DistCoeffs.put(0,1,K2Loc);
                    DistCoeffs.put(0,2,P1Loc);
                    DistCoeffs.put(0,3,P2Loc);
                    DistCoeffs.put(0,4,K3Loc);
                    DistCoeffs.put(0,4,K4Loc);
                    DistCoeffs.put(0,4,K5Loc);
                    DistCoeffs.put(0,4,K6Loc);
                    final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(1);
                    for(final LinkedList<Point>VerticalLine : m_VerticalLines)
                    {
                        if(VerticalLine.size()>=(2*m_ImgSource.rows()/3))
                        {
                            final WeightedObservedPoints obs = new WeightedObservedPoints();
                            final MatOfPoint2f SrcPoints = new MatOfPoint2f();
                            final MatOfPoint2f DstPoints = new MatOfPoint2f();
                            SrcPoints.fromList(VerticalLine);
                            DstPoints.fromList(VerticalLine);
                            Calib3d.undistortPoints(SrcPoints, DstPoints, CameraMatrix, DistCoeffs);
                            for(int i=0; i < VerticalLine.size(); i++)
                            {
                                final double[] DstPoint = DstPoints.get(i, 0);
                                
                                DstPoint[0] = DstPoint[0]*m_FocalLengthXPixel+HxLoc;
                                DstPoint[1] = DstPoint[1]*m_FocalLengthYPixel+HyLoc;
                                obs.add(DstPoint[1], DstPoint[0]);
                            }                            
                            
                            
                            final double[] coeff = fitter.fit(obs.toList());
                            double[][] DataLines = new double[VerticalLine.size()][2];
                            double[][] DataFit = new double[VerticalLine.size()][2];
                            for(int i=0; i < VerticalLine.size(); i++)
                            {
                                final double[] DstPoint = DstPoints.get(i, 0);
                                
                                DstPoint[0] = DstPoint[0]*m_FocalLengthXPixel+HxLoc;
                                DstPoint[1] = DstPoint[1]*m_FocalLengthYPixel+HyLoc;
                                DataLines[i][0]=DstPoint[1];
                                DataLines[i][1]=DstPoint[0];
                                DataFit[i][0]=DstPoint[1];
                                DataFit[i][1]=coeff[0]+coeff[1]*DstPoint[1];
                                final double diff = (coeff[0]+coeff[1]*DstPoint[1]-DstPoint[0]);
                                if(Math.abs(diff)<20.0)
                                {
                                    Error += diff*diff;
                                }
                                
                            }
                            /*JavaPlot p = new JavaPlot();
                            
                            p.set("terminal", "wxt");
                                                        
                            DataSetPlot s = new DataSetPlot(DataLines);
                            DataSetPlot s1 = new DataSetPlot(DataFit);
                            
                            p.addPlot(s);
                            p.addPlot(s1);
                            p.plot();*/
                            
                        }
                        
                    }
                                        
                    return 1.0/Error;
                }
            };

        
        final ObjectiveFunction OFErrFunc = new ObjectiveFunction(ErrFunc);
        
        double Xh=0.5*m_ImgSource.cols();
        double Yh=0.5*m_ImgSource.rows();
        double K1=0.1;
        double K2=0.0;
        double P1=0.0;
        double P2=0.0;
        double K3=0.0;
        double K4=0.0;
        double K5=0.0;
        double K6=0.0;
        
        final MaxEval maxEval = new MaxEval(5000000);
        double[] initials = new double[]{
            Xh,
            Yh,
            K1/*,
            K2,
            P1,
            P2,
            K3,
            K4,
            K5,
            K6*/
        };  
        double[] result_point = new double[initials.length];
        double[] LowerBoundary = new double[initials.length];
        double[] UpperBoundary = new double[initials.length];
        LowerBoundary[0] = 0.5*m_ImgSource.cols()-100.0;
        LowerBoundary[1] = 0.5*m_ImgSource.rows()-100.0;
        LowerBoundary[2] = -1.0;
        /*LowerBoundary[3] = -1.0;
        LowerBoundary[4] = -1.0;
        LowerBoundary[5] = -1.0;
        LowerBoundary[6] = -1.0;
        LowerBoundary[7] = -1.0;
        LowerBoundary[8] = -1.0;
        LowerBoundary[9] = -1.0;*/

        UpperBoundary[0] = 0.5*m_ImgSource.cols()+100.0;
        UpperBoundary[1] = 0.5*m_ImgSource.rows()+100.0;
        UpperBoundary[2] = 1.0;
        /*UpperBoundary[3] = 1.0;
        UpperBoundary[4] = 1.0;
        UpperBoundary[5] = 1.0;
        UpperBoundary[6] = 1.0;
        UpperBoundary[7] = 1.0;
        UpperBoundary[8] = 1.0;
        UpperBoundary[9] = 1.0;*/

        InitialGuess start_values = new InitialGuess(initials);
        SimpleBounds Bounds = new SimpleBounds(LowerBoundary, UpperBoundary);
        BOBYQAOptimizer optim = new BOBYQAOptimizer(2*(initials.length)+1);
        //optim.optimize(OFErrFunc, start_values, maxEval, Bounds);
        
        //final PowellOptimizer optimizer = new PowellOptimizer(1e-4, 1e-2);
        
        try {
            final PointValuePair result = optim.optimize(OFErrFunc, start_values, maxEval, Bounds);
            result_point = result.getPoint();
            
        }
        catch (final TooManyEvaluationsException e) {
            for (int i = 0; i < result_point.length; i++) {
                result_point[i] = Double.NaN;
            }
        }
        /*initials = new double[]{
            Xh,
            Yh,
            result_point[2],
            0.0
        };  
        start_values = new InitialGuess(initials);
        LowerBoundary = new double[initials.length];
        UpperBoundary = new double[initials.length];
        LowerBoundary[0] = 0.5*m_ImgSource.cols()-100.0;
        LowerBoundary[1] = 0.5*m_ImgSource.rows()-100.0;
        LowerBoundary[2] = -1.0;
        LowerBoundary[3] = -1.0;
        

        UpperBoundary[0] = 0.5*m_ImgSource.cols()+100.0;
        UpperBoundary[1] = 0.5*m_ImgSource.rows()+100.0;
        UpperBoundary[2] = 1.0;
        UpperBoundary[3] = 1.0;
        
        Bounds = new SimpleBounds(LowerBoundary, UpperBoundary);
        
        try {
            final PointValuePair result = optim.optimize(OFErrFunc, start_values, maxEval, Bounds);
            result_point = result.getPoint();
            
        }
        catch (final TooManyEvaluationsException e) {
            for (int i = 0; i < result_point.length; i++) {
                result_point[i] = Double.NaN;
            }
        }

        initials = new double[]{
            Xh,
            Yh,
            result_point[2],
            result_point[3],
            0.0
        };  
        start_values = new InitialGuess(initials);
        LowerBoundary = new double[initials.length];
        UpperBoundary = new double[initials.length];
        LowerBoundary[0] = 0.5*m_ImgSource.cols()-100.0;
        LowerBoundary[1] = 0.5*m_ImgSource.rows()-100.0;
        LowerBoundary[2] = -1.0;
        LowerBoundary[3] = -1.0;
        LowerBoundary[4] = -1.0;
        

        UpperBoundary[0] = 0.5*m_ImgSource.cols()+100.0;
        UpperBoundary[1] = 0.5*m_ImgSource.rows()+100.0;
        UpperBoundary[2] = 1.0;
        UpperBoundary[3] = 1.0;
        UpperBoundary[4] = 1.0;
        
        Bounds = new SimpleBounds(LowerBoundary, UpperBoundary);
        //optim = new BOBYQAOptimizer(2*(initials.length)+1);
        try {
            final PointValuePair result = optim.optimize(OFErrFunc, start_values, maxEval, Bounds);
            result_point = result.getPoint();
            
        }
        catch (final TooManyEvaluationsException e) {
            for (int i = 0; i < result_point.length; i++) {
                result_point[i] = Double.NaN;
            }
        }

        /*initials = new double[]{
            Xh,
            Yh,
            result_point[2],
            result_point[3],
            result_point[4],
            0.0
        };  
        start_values = new InitialGuess(initials);
        LowerBoundary = new double[initials.length];
        UpperBoundary = new double[initials.length];
        LowerBoundary[0] = 0.5*m_ImgSource.cols()-100.0;
        LowerBoundary[1] = 0.5*m_ImgSource.rows()-100.0;
        LowerBoundary[2] = -1.0;
        LowerBoundary[3] = -1.0;
        LowerBoundary[4] = -1.0;
        LowerBoundary[5] = -1.0;
        
        UpperBoundary[0] = 0.5*m_ImgSource.cols()+100.0;
        UpperBoundary[1] = 0.5*m_ImgSource.rows()+100.0;
        UpperBoundary[2] = 1.0;
        UpperBoundary[3] = 1.0;
        UpperBoundary[4] = 1.0;
        UpperBoundary[5] = 1.0;
       
        Bounds = new SimpleBounds(LowerBoundary, UpperBoundary);
        optim = new BOBYQAOptimizer(2*(initials.length)+1);
        try {
            final PointValuePair result = optim.optimize(OFErrFunc, start_values, maxEval, Bounds);
            result_point = result.getPoint();
            
        }
        catch (final TooManyEvaluationsException e) {
            for (int i = 0; i < result_point.length; i++) {
                result_point[i] = Double.NaN;
            }
        }

        
        Xh = result_point[0];
        Yh = result_point[1];
        K1 = result_point[2];
        K2 = result_point[3];
        P1 = result_point[4];
        P2 = 0.0;//result_point[5];
        K3 =0.0;//result_point[6];
        K4 = 0.0;//result_point[7];
        K5 = 0.0;//result_point[8];
        K6 = 0.0;//result_point[9];*/

        final Mat CameraMatrix = new Mat(3, 3, 6);
        final Mat DistCoeffs = new Mat(1,5, 6);
        CameraMatrix.put(0, 0, m_FocalLengthXPixel); CameraMatrix.put(0, 1, 0.0); CameraMatrix.put(0, 2, result_point[0]);
        CameraMatrix.put(1, 0, 0.0); CameraMatrix.put(1, 1, m_FocalLengthYPixel); CameraMatrix.put(1, 2, result_point[1]);
        CameraMatrix.put(2, 0, 0.0); CameraMatrix.put(2, 1, 0.0); CameraMatrix.put(2, 2, 1.0);

        DistCoeffs.put(0,0,result_point[2]);
        DistCoeffs.put(0,1,0.0);
        DistCoeffs.put(0,2,0.0);
        DistCoeffs.put(0,3,0.0);
        DistCoeffs.put(0,4,0.0);
        DistCoeffs.put(0,4,0.0);
        DistCoeffs.put(0,4,0.0);
        DistCoeffs.put(0,4,0.0);
        
        Calib3d.undistort(m_ImgSource, m_DistortionFreeImg, CameraMatrix, DistCoeffs);
        Imgcodecs.imwrite("DistortionFreeImage.jpg", m_DistortionFreeImg);
        final Mat ResizeImage = new Mat();
        final Size ScaleSize = new Size(m_ImgSource.cols()/4, m_ImgSource.rows()/4);
        Imgproc.resize(m_DistortionFreeImg, ResizeImage, ScaleSize,0,0,Imgproc.INTER_AREA);
        HighGui.namedWindow("Distortion free image", HighGui.WINDOW_AUTOSIZE);
        HighGui.imshow("Distortion free image", ResizeImage);
        HighGui.waitKey();
    } 
};