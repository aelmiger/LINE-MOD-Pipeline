#include "defines.h"

TemplatePosition::TemplatePosition() {}
TemplatePosition::TemplatePosition(std::string in_s, glm::vec3 in_v, float32 in_rotation, cv::Rect in_boundingBox, uint16 in_depthAtCenter) {
	modelName = in_s;
	positionCam = in_v;
	rotation = in_rotation;
	boundingBox = in_boundingBox;
	depthAtCenter = in_depthAtCenter;
}


ObjectPose::ObjectPose() {}
ObjectPose::ObjectPose(glm::vec3 tra, glm::qua<float32> qua) :
	translation(tra),
	quaternions(qua)
{}
ObjectPose::ObjectPose(glm::vec3 tra, glm::qua<float32> qua, cv::Rect bb) :
	translation(tra),
	quaternions(qua),
	boundingBox(bb)
{}
