/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file cost_functions.cpp
 **/

#include "cost_functions.h"

using namespace std;

namespace cost_functions
{
  // COST FUNCTIONS

  double diff_cost(vector<double> coeff, double duration,
                   std::array<double, 3> goals, std::array<float, 3> sigma,
                   double cost_weight)
  {
    /*
    Penalizes trajectories whose coordinate(and derivatives)
    differ from the goal.
    */
    double cost = 0.0;
    vector<double> evals = evaluate_f_and_N_derivatives(coeff, duration, 2);
    //////////////cout << "26 - Evaluating f and N derivatives Done. Size:" <<
    /// evals.size() << endl;

    for (size_t i = 0; i < evals.size(); i++)
    {
      double diff = fabs(evals[i] - goals[i]);
      cost += logistic(diff / sigma[i]);
    }
    ////////////cout << "diff_coeff Cost Calculated " << endl;
    return cost_weight * cost;
  }

  double collision_circles_cost_spiral(const std::vector<PathPoint> &spiral,
                                       const std::vector<State> &obstacles)
  {
    bool collision{false};
    auto n_circles = CIRCLE_OFFSETS.size();

    for (auto wp : spiral)
    {
      if (collision)
      {
        // LOG(INFO) << " ***** COLLISION DETECTED *********" << std::endl;
        break;
      }
      double cur_x = wp.x;
      double cur_y = wp.y;
      double cur_yaw = wp.theta; // in rad.

      for (size_t c = 0; c < n_circles && !collision; ++c)
      {
        auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * std::cos(cur_yaw);
        auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * std::sin(cur_yaw);

        for (auto obst : obstacles)
        {
          if (collision)
          {
            break;
          }
          auto actor_yaw = obst.rotation.yaw;
          for (size_t c2 = 0; c2 < n_circles && !collision; ++c2)
          {
            auto actor_center_x =
                obst.location.x + CIRCLE_OFFSETS[c2] * std::cos(actor_yaw);
            auto actor_center_y =
                obst.location.y + CIRCLE_OFFSETS[c2] * std::sin(actor_yaw);

            double dist = std::sqrt(((circle_center_x - actor_center_x) * (circle_center_x - actor_center_x)) + ((circle_center_y - actor_center_y) * (circle_center_y - actor_center_y)));

            collision = (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));
          }
        }
      }
    }
    return (collision) ? COLLISION : 0.0;
  }

  double close_to_main_goal_cost_spiral(const std::vector<PathPoint> &spiral,
                                        State main_goal)
  {
    // The last point on the spiral should be used to check how close we are to
    // the Main (center) goal. That way, spirals that end closer to the lane
    // center-line, and that are collision free, will be prefered.
    auto n = spiral.size();

    // distance between last point on spiral and main goal
    auto delta_x = main_goal.location.x - spiral[n - 1].x;
    auto delta_y = main_goal.location.y - spiral[n - 1].y;
    auto delta_z = main_goal.location.z - spiral[n - 1].z;

    auto dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y) +
                          (delta_z * delta_z));

    auto cost = logistic(dist);
    // LOG(INFO) << "distance to main goal: " << dist;
    // LOG(INFO) << "cost (log): " << cost;
    return cost;
  }
} // namespace cost_functions
