#pragma once

/*
 * Copyright (C) <SUB><year> LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
#include <iostream>
#include <vector>

namespace ssc_interface_wrapper
{

  /**
   * \brief Stuct containing the algorithm configuration values for <SUB><package_name>
   */
  struct Config
  {
    int controller_timeout = 1;
    bool use_real_wheel_speed = true;
    bool use_adaptive_gear_ratio = true;
    int command_timeout = 1000;
    double loop_rate = 30.0;

    // Stream operator for this config
    // TODO for USER: Update prints for the added parameters
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "ssc_interface_wrapper::Config { " << std::endl
           << "controller_timeout: " << c.controller_timeout << std::endl
           << "use_real_wheel_speed: "<< c.use_real_wheel_speed << std::endl
           << "use_adaptive_gear_ratio: "<< c.use_adaptive_gear_ratio <<std::endl
           << "command_timeout: "<< c.command_timeout << std::endl
           << " loop_rate: " << c.loop_rate << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // ssc_interface_wrapper