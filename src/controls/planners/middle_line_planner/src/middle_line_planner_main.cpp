/*******************************************************************************
 * AMZ Driverless Project                                                      *
 * Copyright (c) 2024-2025                                                     *
 * Authors:                                                                    *
 *   - Nazim Ozan Yasar <nyasar@ethz.ch>                                       *
 *   - Mattia Mangili <mangilim@ethz.ch>                                       *
 *   - Kevin Gabriel <kegabriel@ethz.ch>                                       *
 *   - Vincent Rasse <vrasse@ethz.ch>                                          *
 *   - Audrey Kubler <akubler@ethz.ch>                                         *
 *   - Sinan Laloui <slaloui@ethz.ch>                                          *
 *   - Alexander Terrail <aterrail@ethz.ch>                                    *
 *                                                                             *
 * All rights reserved.                                                        *
 *                                                                             *
 * Unauthorized copying of this file, via any medium, is strictly prohibited.  *
 * Proprietary and confidential.                                               *
 ******************************************************************************/

#include "middle_line_planner/middle_line_planner.hpp"
 
 // Helper function to obtain a formatted timestamp string ("YYYY_MM_DD-HH_MM")
 std::string getCurrentTimestamp() {
   std::time_t now = std::time(nullptr);
   char buffer[20]; // Ensure this is large enough for the formatted string and null terminator
   std::strftime(buffer, sizeof(buffer), "%Y_%m_%d-%H_%M", std::localtime(&now));
   return std::string(buffer);
 }
 
 int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   std::shared_ptr<MiddleLinePlanner> middle_line_planner = std::make_shared<MiddleLinePlanner>();
  
   // Get a timestamp for naming the profiler output file
   const std::string timestamp = getCurrentTimestamp();
 
   // Create an executor and add both nodes for concurrent execution
   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(middle_line_planner);
   executor.spin();
 
   rclcpp::shutdown();
 
   return 0;
 }