#include "defines.h"
#include <utility>

TemplatePosition::TemplatePosition()
{
}

TemplatePosition::TemplatePosition(std::string in_s, glm::vec3 in_v, float in_rotation, cv::Rect in_boundingBox,
	uint16_t in_depthAtCenter)
{
	modelName = std::move(in_s);
	positionCam = in_v;
	rotation = in_rotation;
	boundingBox = std::move(in_boundingBox);
	depthAtCenter = in_depthAtCenter;
}

ObjectPose::ObjectPose()
{
}

ObjectPose::ObjectPose(glm::vec3 tra, glm::qua<float> qua) :
	translation(tra),
	quaternions(qua)
{
}

ObjectPose::ObjectPose(glm::vec3 tra, glm::qua<float> qua, cv::Rect bb) :
	translation(tra),
	quaternions(qua),
	boundingBox(std::move(bb))
{
}