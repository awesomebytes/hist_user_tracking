/*!
  File:         test_conditional_particle_filter.cpp
  Author:       Arnaud Ramey <arnaud.a.ramey@gmail.com>
  Date:         2012/1
  ______________________________________________________________________________

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  ______________________________________________________________________________

  */

#include "utils/debug/error.h"
#include "conditional_particle_filter_laser_ros.h"

void test_select_index_with_probas() {
  srand(time(NULL));
  float probas[] = {0.1, 0.2, 0.4, 0.3};
  int occurences[] = {0, 0, 0, 0};

  for (unsigned int var = 0; var < 1000; ++var) {
    unsigned int index = select_index_with_probas::select(probas, probas + 4);
    ++ occurences[index];
    //maggiePrint("index:%i", index);
  } // end loop var

  maggiePrint("occurences:%i, %i, %i, %i",
              occurences[0], occurences[1], occurences[2], occurences[3]);
} // end test_select_index_with_probas();

////////////////////////////////////////////////////////////////////////////////

void test_ros(int argc, char** argv) {
  ros::init(argc, argv, "test_conditional_particle_filter");
  conditional_particle_filter_laser::ConditionalParticleFilterLaserRos filter;
  ros::spin();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  maggieDebug2("main()");
  //test_select_index_with_probas();
  test_ros(argc, argv);
  return 0;
}


