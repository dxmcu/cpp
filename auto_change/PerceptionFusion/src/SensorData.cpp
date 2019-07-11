#include "SensorData.h"


CSensorData::CSensorData()
{
	vehicle_angle = 0.0;
}


CSensorData::~CSensorData()
{
}
DeepBlue::Extent2D CSensorData::GetObjectEnvlope()
{
	DeepBlue::Extent2D extent;

	for (int i = 0; i < 4; i++)
	{
		if (extent.min_x > measured_corners[i](0))
			extent.min_x = measured_corners[i](0);
		if (extent.max_x < measured_corners[i](0))
			extent.max_x = measured_corners[i](0);

		if (extent.min_y > measured_corners[i](1))
			extent.min_y = measured_corners[i](1);
		if (extent.max_y < measured_corners[i](1))
			extent.max_y = measured_corners[i](1);

	}

	return extent;

}
