#pragma once

#include <vector>
#include <cfloat>
#include "common_defs.h"
namespace DeepBlue{

	typedef enum {
		enumGEOM_NONE = 0,
		enumGEOM_POINT = 1,
		enumGEOM_LINE = 2,
		enumGEOM_POLYGON = 3
	}GIS_GEOMETRY_TYPE;

	typedef enum {
		enumSHAPE_NOTSET = 1000,
		enumSHAPE_POINT = 1001,
		enumSHAPE_MULTIPOINTS = 1002,
		enumSHAPE_LINE = 1003,
		enumSHAPE_POLYGON = 1004,
		enumSHAPE_ANNOTATION = 1005,
		enumSHAPE_CELL = 1006,
		enumSHAPE_IMAGELIB = 1007,
		enumSHAPE_VECTORLIB = 1008
	}GIS_SHAPE_TYPE;


	typedef enum {
		enumINTERSECT = 1,
		enumSEPARATED = 2,
		enumTOUCHED = 3
	}GIS_SPATIAL_RELATION;

	class GeomPoint;
	class Envelope{
	public:
		double left;
		double right;
		double top;
		double bottom;
	public:
		Envelope();
		Envelope(double l, double r, double t, double b);

		double getWidth();
		double getHeight();

		double getCenterX();
		double getCenterY();
	public:
		bool isValid();
		bool contains(Envelope e);
		bool contains(double x, double y);
		bool contains(double x1, double y1, double x2, double y2);
		bool intersected(double x1, double y1, double x2, double y2);
		bool intersected(Envelope ev);
		bool is2Small(double threshold);
		int  getIntersectPoints(double x1, double y1, double x2, double y2, GeomPoint& pt1, GeomPoint& pt2);
	};
	struct Extent2D{
		double min_x;
		double max_x;
		double min_y;
		double max_y;
		Extent2D(){
			min_x = DBL_MAX;
			min_y = DBL_MAX;
			max_x = DBL_MIN;
			max_y = DBL_MIN;
		};
		bool valid()
		{
			if (min_x > max_x || min_y > max_y || fabs(max_x-min_x) < 1e-10 || fabs(max_y-min_y) < 1e-10)
				return false;
			return true;
		}
		double area()
		{
			return (max_x - min_x)*(max_y - min_y);
		}
	};
	class Geometry {
	protected:
		GIS_GEOMETRY_TYPE  geometryType;
		int id;
		int tagId;

	public:
		Envelope envlope;

		Geometry();

		GIS_GEOMETRY_TYPE getGeometryType();

		int getId();
		void setId(int idx);
		int getTagID();
		void setTagID(int tag);
	};

	class GeomPoint : public Geometry{
	public:
		double x;
		double y;

	public:
		GeomPoint();
		GeomPoint(double nx, double ny);
	};

	class GeomLine : public Geometry{
	private:
		std::vector<GeomPoint*>  points;

	public:
		GeomLine();
		~GeomLine();
		GeomLine(double*  px, double*  py, int pointscount);

		int getSize();
		GeomPoint* getRef(int index);
		void add(double x, double y);
		void insert(int index, double x, double y);
		void remove(int index);
	};

	class GeomPolygon : public Geometry {
	private:
		std::vector<GeomPoint*>  points;

	public:
		GeomPolygon();
		~GeomPolygon();
		GeomPolygon(double*  px, double*  py, int pointscount);

		int getSize();
		GeomPoint* getRef(int index);
		void add(double x, double y);
		void insert(int index, double x, double y);
		void remove(int index);
	};

	//used for device rectangle
	class Rectangle {
	public:
		int left;
		int top;
		int right;
		int bottom;
	public:
		Rectangle();
		int width();
		int height();
	};
}

