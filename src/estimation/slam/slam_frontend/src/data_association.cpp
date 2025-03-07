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

#include "slam_frontend/data_association.hpp"

namespace slam {

DataAssociation::DataAssociation(std::shared_ptr<SlamNode> node_handle) : node_handle_(node_handle) {}

DataAssociation::~DataAssociation() {}

} // end namespace slam
