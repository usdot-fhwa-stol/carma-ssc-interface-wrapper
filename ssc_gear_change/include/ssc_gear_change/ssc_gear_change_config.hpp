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

namespace ssc_gear_change
{

  /**
   * \brief Stuct containing the algorithm configuration values for <SUB><package_name>
   */
  struct Config
  {
    double delay_time_ = 1.0;

    // Stream operator for this config
    // TODO for USER: Update prints for the added parameters
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "ssc_gear_change::Config { " << std::endl
           << "delay_time_: " << c.delay_time_ << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // ssc_gear_change