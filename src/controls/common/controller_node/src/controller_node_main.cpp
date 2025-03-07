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

 #include "controller_node/controller_node.hpp"
 
 // Helper function to obtain a formatted timestamp string ("YYYY_MM_DD-HH_MM")
 std::string getCurrentTimestamp() {
   std::time_t now = std::time(nullptr);
   char buffer[20]; // Ensure this is large enough for the formatted string and null terminator
   std::strftime(buffer, sizeof(buffer), "%Y_%m_%d-%H_%M", std::localtime(&now));
   return std::string(buffer);
 }
 
 int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   std::shared_ptr<ControllerNode> controller_node = std::make_shared<ControllerNode>("controller_node");
 
   // Check for profiling parameter and enable profiler if requested
   bool profiling_enabled = controller_node->get_parameter("profiling_enabled").as_bool();
   if (profiling_enabled) {
     EASY_PROFILER_ENABLE;
   }
 
   // Get a timestamp for naming the profiler output file
   const std::string timestamp = getCurrentTimestamp();
 
   // Create an executor and add both nodes for concurrent execution
   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(controller_node);
   executor.spin();
 
   rclcpp::shutdown();
 
   // After shutdown, if profiling was enabled, dump the collected data to a file
   if (profiling_enabled) {
     std::string prof_filename = "./profiler/controller_node" + timestamp + ".prof";
     profiler::dumpBlocksToFile(prof_filename.c_str());
     RCLCPP_INFO_STREAM(controller_node->get_logger(), "Profiling data saved to " << prof_filename);
   }
 
   return 0;
 }