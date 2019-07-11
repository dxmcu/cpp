#include "VectorUtil.h"
#include "data_struct_define.h"

using namespace std;
double DeepBlue::CVectorUtil::dbl_precesion = 1e-8;

DeepBlue::CVectorUtil::CVectorUtil()
{
}


DeepBlue::CVectorUtil::~CVectorUtil()
{
}
/**
* used to compute azimuth angle by device coordinates
* @param x0   device x
* @param y0   device y
* @return     azimuth angle -  angle between vector and positive Y by clockwize direction from positive Y
*/
double DeepBlue::CVectorUtil::ComputeAzimuthAngle(double x0, double y0)
{
	if (fabs(x0) < dbl_precesion){
		if (y0 > 0)
			return 0;
		else
			return PI / 2;
	}
	if (x0 > 0 && y0 >= 0){
		return PI / 2 - atan((double)fabs(y0 / x0));
	}
	if (x0 > 0 && y0 < 0){
		return PI / 2 + atan((double)fabs(y0 / x0));
	}
	if (x0 < 0 && y0 <0){
		return 3 * PI / 2 - atan((double)fabs(y0 / x0));
	}
	if (x0 < 0 && y0 > 0){
		return 3 * PI / 2 + atan((double)fabs(y0 / x0));
	}
	return 0;
}
bool  DeepBlue::CVectorUtil::EnvlopeIntersected(Envelope e1, Envelope e2)
{
	if (e1.left > e2.right || e1.right < e2.left || e1.top < e2.bottom || e1.bottom > e2.top){
		return false;
	}
	return true;
}
bool  DeepBlue::CVectorUtil::Extent2dIntersected(Extent2D e1, Extent2D e2)
{
	if (e1.min_x > e2.max_x || e1.max_x < e2.min_x || e1.max_y < e2.min_y || e1.min_y > e2.max_y){
		return false;
	}
	return true;
}


/**
* to detect the geometry relation of two line segments
* @param x1
* @param y1
* @param x2
* @param y2
* @param scale : the scale factor to make coordinates long
* @return the relation of these two segment
* 0 - not intersect
* 1 - intersect
* 2 - coline
*/
int DeepBlue::CVectorUtil::TwoLineSegmentsRelDetect(double px1, double py1, double px2, double py2, double qx1, double qy1, double qx2, double qy2, GeomPoint& pt)
{
	int result = 0;

	//decide whether the envlopes of two segs are intersected
	if (min(px1, px2) > max(qx1, qx2) || max(px1, px2)<min(qx1, qx2))
		return result;
	if (min(py1, py2) > max(qy1, qy2) || max(py1, py2)<min(qy1, qy2))
		return result;

	//conclude whether two segs are intersected by cross product.
	double p1q1 = (px1 - qx1)*(qy2 - qy1) - (qx2 - qx1)*(py1 - qy1);
	double p2q1 = (px2 - qx1)*(qy2 - qy1) - (qx2 - qx1)*(py2 - qy1);
	double q1p1 = (qx1 - px1)*(py2 - py1) - (px2 - px1)*(qy1 - py1);
	double q2p1 = (qx2 - px1)*(py2 - py1) - (px2 - px1)*(qy2 - py1);
	double p1q1p2q1 = p1q1*p2q1;
	double q1p1q2p1 = q1p1*q2p1;

	//that the corss product is equivalent to 0 means two segs are in the same line
	if ((fabs(p1q1)<dbl_precesion&&fabs(p2q1) < dbl_precesion) || ((fabs(q1p1) < dbl_precesion&&fabs(q2p1) < dbl_precesion))) {
		return 2;
	}

	//two segs are intersected and touched
	if (fabs(p1q1)<dbl_precesion){
		pt.x = px1;
		pt.y = py1;
		return 1;
	}
	if (fabs(p2q1)<dbl_precesion){
		pt.x = px2;
		pt.y = py2;
		return 1;
	}
	if (fabs(q1p1)<dbl_precesion){
		pt.x = qx1;
		pt.y = qy1;
		return 1;
	}
	if (fabs(q2p1)<dbl_precesion){
		pt.x = qx2;
		pt.y = qy2;
		return 1;
	}

	//two segs are intersected
	if (p1q1p2q1 < 0 && q1p1q2p1 < 0) {
		GeomPoint rpt = GetCrossPoint(px1, py1, px2, py2, qx1, qy1, qx2, qy2);
		pt.x = rpt.x;
		pt.y = rpt.y;
		return 1;
	}

	return result;
}

/**
* get the crosspoint of two segs, the crosspoint must be exist
* @param x1
* @param y1
* @param x2
* @param y2
* @param x3
* @param y3
* @param x4
* @param y4
* 	 * @return
*/
DeepBlue::GeomPoint  DeepBlue::CVectorUtil::GetCrossPoint(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
	GeomPoint pt;
	double k1, k2;

	if (fabs(x1 - x2)< dbl_precesion){ // the first segments is vertical vector
		pt.x = x1;
		pt.y = (y4 - y3) / (x4 - x3)*(x1 - x3) + y3;
		return pt;
	}

	if (fabs(x3 - x4)< dbl_precesion){ // the second segments is vertical vector
		pt.x = x3;
		pt.y = (y2 - y1) / (x2 - x1)*(x3 - x1) + y1;
		return pt;
	}


	//both segs are not vertical
	k2 = (y4 - y3) / (x4 - x3);
	k1 = (y2 - y1) / (x2 - x1);
	if (fabs(k1 - k2)<dbl_precesion){

	}
	else{
		pt.x = (y3 - y1 - x3*k2 + k1*x1) / (k1 - k2);
		pt.y = k2*(pt.x - x3) + y3;
	}
	return GeomPoint(pt.x, pt.y);
}
bool DeepBlue::CVectorUtil::IsSegmentsDuplicated(double px1, double py1, double px2, double py2, double qx1, double qy1, double qx2, double qy2)
{
	if ((fabs(min(px1, px2) - min(qx1, qx2))<dbl_precesion) &&
		(fabs(max(px1, px2) - max(qx1, qx2))<dbl_precesion) &&
		(fabs(min(py1, py2) - min(qy1, qy2))<dbl_precesion) &&
		(fabs(max(py1, py2) - max(qy1, qy2))<dbl_precesion))
		return true;

	return false;
}
/**
* conclude whether the two segs are parallel
*
* @param px1
* @param py1
* @param px2
* @param py2
* @param qx1
* @param qy1
* @param qx2
* @param qy2
* @return
*/
bool DeepBlue::CVectorUtil::IsSegmentsParallel(double px1, double py1, double px2, double py2, double qx1, double qy1, double qx2, double qy2)
{
	double newpx1 = fabs(px2 - px1);
	double newpy1 = fabs(py2 - py1);

	double newqx1 = fabs(qx1 - qx2);
	double newqy1 = fabs(qy1 - qy2);

	//compute cross product of the two segments
	double pXq = (newpx1*newqy1 - newpy1*newqx1);

	if (fabs(pXq) < dbl_precesion)
		return true;

	return false;
}
bool DeepBlue::CVectorUtil::Is2PointsOverlap(double ptx1, double pty1, double ptx2, double pty2)
{
	if (fabs(ptx1 - ptx2)<dbl_precesion && fabs(pty1 - pty2)<dbl_precesion)
		return true;
	return false;
}
