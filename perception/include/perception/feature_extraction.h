#ifndef _PERCEPTION_FEATURE_EXTRACTION_H_
#define _PERCEPTION_FEATURE_EXTRACTION_H_

#include "perception/object.h"
#include "perception_msgs/ObjectFeatures.h"

namespace perception {
void ExtractFeatures(const Object& object,
                     perception_msgs::ObjectFeatures* features);

void ExtractSizeFeatures(const Object& object,
                         perception_msgs::ObjectFeatures* features);

void ExtractColorFeatures(const Object& object,
                          perception_msgs::ObjectFeatures* features);
}  // namespace perception

#endif  // _PERCEPTION_FEATURE_EXTRACTION_H_
