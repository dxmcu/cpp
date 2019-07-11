#include "Geometry.h"

#include <iostream>
#include <limits>
#include <vector>
#include "math.h"
#include <exception>

#include "VectorUtil.h"

using namespace std;
using namespace DeepBlue;


//Implementation of Envlope
DeepBlue::Envelope::Envelope()
{
  left = numeric_limits<float>::max();
  right = -numeric_limits<float>::max();
  top = -numeric_limits<float>::max();
  bottom = numeric_limits<float>::max();
}
DeepBlue::Envelope::Envelope(double l, double r, double t, double b){
  left = l;
  right = r;
  top = t;
  bottom = b;
}
bool DeepBlue::Envelope::isValid()
{
  try{
    if ((right - left)<0 || (top - bottom)<0)
      return false;
    else
      return true;
  }
  catch (exception& e){
    cout << e.what() << endl;
    return false;
  }
}

double DeepBlue::Envelope::getWidth()
{
  return right - left;
}
double DeepBlue::Envelope::getHeight()
{
  return top - bottom;
}
double DeepBlue::Envelope::getCenterX()
{
  return (left + (right - left) / 2);
}
double DeepBlue::Envelope::getCenterY()
{
  return (top - (top - bottom) / 2);
}
bool DeepBlue::Envelope::contains(Envelope e)
{
  if (e.left >= left && e.right <= right && e.top <= top && e.bottom >= bottom)
    return true;
  return false;
}
bool DeepBlue::Envelope::contains(double x, double y)
{
  if (x > left && x < right && y < top && y > bottom)
    return true;
  return false;
}
bool DeepBlue::Envelope::contains(double x1, double y1, double x2, double y2)
{
  if (min(x1, x2) > left && max(x1, x2) < right && max(y1, y2) < top && min(y1, y2) > bottom)
    return true;
  return false;
}
bool DeepBlue::Envelope::intersected(double x1, double y1, double x2, double y2)
{
  if (max(x1, x2) < left || min(x1, x2) > right || min(y1, y2) > top || max(y1, y2) < bottom)
    return false;
  return true;
}
bool DeepBlue::Envelope::intersected(Envelope ev)
{
  if (ev.left > right || ev.right < left || ev.top < bottom || ev.bottom > top)
    return false;
  return true;
}
bool Envelope::is2Small(double threshold)
{
  if (fabs(left - right)<threshold || fabs(top - bottom) <threshold)
    return true;
  return false;
}
//get intersect points between line segment and envelope, the max number of intersect points is 2.
int DeepBlue::Envelope::getIntersectPoints(double x1, double y1, double x2, double y2, GeomPoint& pt1, GeomPoint& pt2)
{
  int count = 0;
  GeomPoint pt;
  if (CVectorUtil::TwoLineSegmentsRelDetect(x1, y1, x2, y2, left, top, right, top, pt) == 1){
    pt1.x = pt.x;
    pt1.y = pt.y;
    count++;
  }
  if (CVectorUtil::TwoLineSegmentsRelDetect(x1, y1, x2, y2, right, top, right, bottom, pt) == 1){
    count++;
    if (count == 1){
      pt1.x = pt.x;
      pt1.y = pt.y;
    }
    if (count == 2){
      pt2.x = pt.x;
      pt2.y = pt.y;
      goto dosort;
    }
  }
  if (CVectorUtil::TwoLineSegmentsRelDetect(x1, y1, x2, y2, right, bottom, left, bottom, pt) == 1){
    count++;
    if (count == 1){
      pt1.x = pt.x;
      pt1.y = pt.y;
    }
    if (count == 2){
      pt2.x = pt.x;
      pt2.y = pt.y;
      goto dosort;
    }
  }
  if (CVectorUtil::TwoLineSegmentsRelDetect(x1, y1, x2, y2, left, bottom, left, top, pt) == 1){
    count++;
    if (count == 1){
      pt1.x = pt.x;
      pt1.y = pt.y;
    }
    if (count == 2){
      pt2.x = pt.x;
      pt2.y = pt.y;
      goto dosort;
    }
  }
  double tmpx, tmpy;
dosort: if (count == 2){ // sort the two intersect points according to the line segment's direction
    if (fabs(x2 - x1)>CVectorUtil::dbl_precesion){
      if ((x2 > x1 && pt2.x < pt1.x) || (x2 < x1 && pt2.x > pt1.x)){
        tmpx = pt1.x;
        tmpy = pt1.y;
        pt1.x = pt2.x;
        pt1.y = pt2.y;
        pt2.x = tmpx;
        pt2.y = tmpy;
      }
    }
    else{
      if ((y2 > y1 && pt2.y < pt1.y) || (y2 < y1 && pt2.y > pt1.y)){
        tmpx = pt1.x;
        tmpy = pt1.y;
        pt1.x = pt2.x;
        pt1.y = pt2.y;
        pt2.x = tmpx;
        pt2.y = tmpy;
      }
    }
  }
  return count;
}
//////////////////////////////////////////////////////

//implementation of Geometry
DeepBlue::Geometry::Geometry()
{
  id = 0;
  tagId = 0;
  geometryType = DeepBlue::enumGEOM_NONE;
}

DeepBlue::GIS_GEOMETRY_TYPE DeepBlue::Geometry::getGeometryType(){
  return geometryType;
}
int DeepBlue::Geometry::getId()
{
  return id;
}
void DeepBlue::Geometry::setId(int idx)
{
  id = idx;
}
int DeepBlue::Geometry::getTagID()
{
  return tagId;
}
void DeepBlue::Geometry::setTagID(int tag)
{
  tagId = tag;
}
//////////////////////////////////////////////////////

//implementation of GeometryPoint
DeepBlue::GeomPoint::GeomPoint()
{
  geometryType = DeepBlue::enumGEOM_POINT;
  x = 0.0;
  y = 0.0;
}
DeepBlue::GeomPoint::GeomPoint(double nx, double ny)
{
  geometryType = DeepBlue::enumGEOM_POINT;
  x = nx;
  y = ny;
}
////////////////////////////////////////////////////////

//implementation of GeometryLine
DeepBlue::GeomLine::GeomLine()
{
  geometryType = DeepBlue::enumGEOM_LINE;
}
DeepBlue::GeomLine::~GeomLine()
{
  std::vector<GeomPoint*>::iterator pos = points.begin();
  int c = points.size();
  for (int i = 0; i < c; i++){
    delete points[i];
  }
  points.empty();
}
DeepBlue::GeomLine::GeomLine(double*  px, double*  py, int pointscount)
{
  if (pointscount <= 0)
    return;
  geometryType = DeepBlue::enumGEOM_LINE;

  for (int i = 0; i < pointscount; i++){
    points.push_back(new GeomPoint(px[i], py[i]));
    envlope.left = min(envlope.left, px[i]);
    envlope.right = max(envlope.right, px[i]);
    envlope.top = max(envlope.top, py[i]);
    envlope.bottom = min(envlope.bottom, py[i]);
  }
}
int DeepBlue::GeomLine::getSize()
{
  return points.size();
}

DeepBlue::GeomPoint* DeepBlue::GeomLine::getRef(int index)
{
  if (index<0 || index >= (int)points.size())
    return NULL;
  return points[index];
}

void DeepBlue::GeomLine::add(double x, double y)
{
  points.push_back(new GeomPoint(x, y));
  envlope.left = min(envlope.left, x);
  envlope.right = max(envlope.right, x);
  envlope.top = max(envlope.top, y);
  envlope.bottom = min(envlope.bottom, y);
}
void DeepBlue::GeomLine::insert(int index, double x, double y){
  envlope.left = min(envlope.left, x);
  envlope.right = max(envlope.right, x);
  envlope.top = max(envlope.top, y);
  envlope.bottom = min(envlope.bottom, y);
  std::vector<GeomPoint*>::iterator pos = points.begin();
  pos += index;
  points.insert(pos, new GeomPoint(x, y));
}
void DeepBlue::GeomLine::remove(int index)
{
  std::vector<GeomPoint*>::iterator pos = points.begin();
  pos += index;
  delete points[index];
  points.erase(pos);
}
//////////////////////////////////////////////////////

//implementation of geometrypolygon
DeepBlue::GeomPolygon::GeomPolygon()
{
  geometryType = DeepBlue::enumGEOM_LINE;
}
DeepBlue::GeomPolygon::~GeomPolygon()
{
  std::vector<GeomPoint*>::iterator pos = points.begin();
  int c = points.size();
  for (int i = 0; i < c; i++){
    delete points[i];
  }
  points.empty();
}

DeepBlue::GeomPolygon::GeomPolygon(double*  px, double*  py, int pointscount)
{
  if (pointscount <= 0)
    return;
  geometryType = DeepBlue::enumGEOM_LINE;

  for (int i = 0; i < pointscount; i++){
    points.push_back(new GeomPoint(px[i], py[i]));
    envlope.left = min(envlope.left, px[i]);
    envlope.right = max(envlope.right, px[i]);
    envlope.top = max(envlope.top, py[i]);
    envlope.bottom = min(envlope.bottom, py[i]);
  }
}
int DeepBlue::GeomPolygon::getSize()
{
  return points.size();
}

DeepBlue::GeomPoint* DeepBlue::GeomPolygon::getRef(int index)
{
  if (index<0 || index >= (int)points.size())
    return NULL;
  return points[index];
}

void DeepBlue::GeomPolygon::add(double x, double y)
{
  points.push_back(new GeomPoint(x, y));
  envlope.left = min(envlope.left, x);
  envlope.right = max(envlope.right, x);
  envlope.top = max(envlope.top, y);
  envlope.bottom = min(envlope.bottom, y);
}

void DeepBlue::GeomPolygon::insert(int index, double x, double y){
  envlope.left = min(envlope.left, x);
  envlope.right = max(envlope.right, x);
  envlope.top = max(envlope.top, y);
  envlope.bottom = min(envlope.bottom, y);
  std::vector<GeomPoint*>::iterator pos = points.begin();
  pos += index;
  points.insert(pos, new GeomPoint(x, y));
}

void DeepBlue::GeomPolygon::remove(int index)
{
  std::vector<GeomPoint*>::iterator pos = points.begin();
  pos += index;
  delete points[index];
  points.erase(pos);
}
//////////////////////////////////////////////////////////////////////


//Rectangle start here
Rectangle::Rectangle()
{
  left = numeric_limits<int>::max();
  right = -numeric_limits<int>::max();
  top = numeric_limits<int>::max();
  bottom = -numeric_limits<int>::max();
}
int Rectangle::width()
{
  return right - left;
}
int Rectangle::height()
{
  return bottom - top;
}
////////////////////////////////////////////////////////////////////////
