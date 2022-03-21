#pragma once

/*
 * Copyright (C) 2022 LEIDOS.
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
   * \brief Stuct containing the algorithm configuration values for ssc_interface_wrapper
   */
  struct Config
  {
    // Timeout for command receipt in seconds
    double controller_timeout = 1;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "ssc_interface_wrapper::Config { " << std::endl
           << "controller_timeout: "<< c.controller_timeout << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // ssc_interface_wrapper