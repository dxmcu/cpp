#pragma once
#include "Geometry.h"
namespace DeepBlue{
	class CVectorUtil
	{
	public:
		CVectorUtil();
		~CVectorUtil();
	public:
		static double dbl_precesion;
	public:
		static double      ComputeAzimuthAngle(double x0, double y0);
		static bool        EnvlopeIntersected(Envelope e1, Envelope e2);
		static bool        Extent2dIntersected(Extent2D e1, Extent2D e2);
		static int         TwoLineSegmentsRelDetect(double px1, double py1, double px2, double py2, double qx1, double qy1, double qx2, double qy2, GeomPoint& pt);
		static bool        IsSegmentsDuplicated(double px1, double py1, double px2, double py2, double qx1, double qy1, double qx2, double qy2);
		static bool        IsSegmentsParallel(double px1, double py1, double px2, double py2, double qx1, double qy1, double qx2, double qy2);
		static GeomPoint   GetCrossPoint(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
		static bool		   Is2PointsOverlap(double ptx1, double pty1, double ptx2, double pty2);
	};
}


