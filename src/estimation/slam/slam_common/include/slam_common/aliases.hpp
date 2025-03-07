/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2024-2025 Authors:
 *   - Yee Hsien Quek <yequek@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

#include <vector>

#include "autonomous_msgs/msg/cone.hpp"

namespace slam {

/*
 * Data type to represent a map of Cones.
 */
typedef std::vector<autonomous_msgs::msg::Cone> ConeMap;

} // namespace slam
