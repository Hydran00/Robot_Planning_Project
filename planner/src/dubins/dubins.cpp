#include <iostream>
#include <fstream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "planner/dubins/dubins.h"

DubinsPath::DubinsPath(std::vector<double> start, std::vector<double> end, double r)
    : _s({start}), _e({end}), _r(r) {}


std::tuple<std::vector<double>, std::vector<double>> gen_path(
    const std::vector<double> &s, const std::vector<std::vector<double>> &path, double r = 1.0, double step = 0.1)
{
    auto calc_TurnCenter = [](const std::vector<double> &point, char dir, double r) -> std::tuple<double, double>
    {
        double ang;
        if (dir == 'l')
        {
            ang = point[2] + M_PI / 2;
        }
        else if (dir == 'r')
        {
            ang = point[2] - M_PI / 2;
        }
        else
        {
            return {0.0, 0.0}; // Return an invalid point for an unknown direction
        }
        double x = point[0] + std::cos(ang) * r;
        double y = point[1] + std::sin(ang) * r;

        return {x, y};
    };

    std::vector<double> r_x;
    std::vector<double> r_y;
    std::vector<double> ps_x;
    std::vector<double> ps_y;
    std::vector<double> start = s;
    double yaw = s[2];

    for (const auto &p : path)
    {
        if (p[0] == 's')
        {
            for (double l = 0.0; l < p[1]; l += 0.5)
            {
                ps_x.push_back(start[0] + std::cos(yaw) * l);
                ps_y.push_back(start[1] + std::sin(yaw) * l);
            }
            ps_x.push_back(start[0] + std::cos(yaw) * p[1]);
            ps_y.push_back(start[1] + std::sin(yaw) * p[1]);
            r_x.insert(r_x.end(), ps_x.begin(), ps_x.end());
            r_y.insert(r_y.end(), ps_y.begin(), ps_y.end());
        }
        else
        {
            auto center = calc_TurnCenter(start, static_cast<char>(p[0]), r);
            double ang_start = std::atan2(start[1] - std::get<1>(center), start[0] - std::get<0>(center));
            double ang_end = ang_start + p[1] * (p[0] == 'l' ? 1 : -1);
            for (double ang = ang_start; (p[0] == 'l' ? ang <= ang_end : ang >= ang_end); ang += (p[0] == 'l' ? step : -step))
            {
                ps_x.push_back(std::get<0>(center) + std::cos(ang) * r);
                ps_y.push_back(std::get<1>(center) + std::sin(ang) * r);
            }
            ps_x.push_back(std::get<0>(center) + std::cos(ang_end) * r);
            ps_y.push_back(std::get<1>(center) + std::sin(ang_end) * r);
            r_x.insert(r_x.end(), ps_x.begin(), ps_x.end());
            r_y.insert(r_y.end(), ps_y.begin(), ps_y.end());

            yaw = start[2] + p[1] * (p[0] == 'l' ? 1 : -1);
        }
        start = {ps_x.back(), ps_y.back(), yaw};
        ps_x.clear();
        ps_y.clear();
    }
    return {r_x, r_y};
}

std::vector<std::vector<std::vector<double>>> DubinsPath::calc_paths()
{
    auto le = calc_end();
    auto types = {&DubinsPath::calc_lsl_from_origin,
                  &DubinsPath::calc_rsr_from_origin,
                  &DubinsPath::calc_lsr_from_origin,
                  &DubinsPath::calc_rsl_from_origin,
                  &DubinsPath::calc_rlr_from_origin,
                  &DubinsPath::calc_lrl_from_origin};
    for (auto t : types)
    {
        auto path = (this->*t)(le);
        if (!path.empty())
        {
            _paths.push_back(path);
        }
    }
    return _paths;
}

std::tuple<std::vector<std::vector<double>>,double> DubinsPath::get_shortest_path_cost()
{
    // FULL_PATH, TOTAL COST, SYMBOLIC PATHS
    std::tuple<std::vector<std::vector<double>>, double> shortest_path_cost;
    std::get<1>(shortest_path_cost) = std::numeric_limits<double>::infinity();
    for (auto &path : _paths)
    {
        double cost = 0;
        for (auto &p : path)
        {
            cost += (p[0] == 's') ? p[1] : p[1] * _r;
        }
        if (cost < std::get<1>(shortest_path_cost))
        {
            std::get<0>(shortest_path_cost) = path;
            std::get<1>(shortest_path_cost) = cost;
        }
    }
    return shortest_path_cost;
}

std::vector<double> DubinsPath::calc_end()
{
    double ex = _e[0][0] - _s[0][0];
    double ey = _e[0][1] - _s[0][1];

    double lex = cos(_s[0][2]) * ex + sin(_s[0][2]) * ey;
    double ley = -sin(_s[0][2]) * ex + cos(_s[0][2]) * ey;
    double leyaw = _e[0][2] - _s[0][2];
    lex = lex / _r;
    ley = ley / _r;
    return {lex, ley, leyaw};
}

double DubinsPath::mod2pi(double theta)
{
    return theta - 2.0 * M_PI * floor(theta / (2.0 * M_PI));
}

std::vector<std::vector<double>> DubinsPath::calc_lsl_from_origin(std::vector<double> e)
{
    double x_ = e[0] - sin(e[2]);
    double y_ = e[1] - 1 + cos(e[2]);

    double u = sqrt(pow(x_, 2) + pow(y_, 2));
    double t = mod2pi(atan2(y_, x_));
    double v = mod2pi(e[2] - t);
    return {{'l', t}, {'s', u * _r}, {'l', v}};
}

std::vector<std::vector<double>> DubinsPath::calc_rsr_from_origin(std::vector<double> e)
{
    auto path = calc_lsl_from_origin(e);
    path[0][0] = 'r';
    path[2][0] = 'r';
    return path;
}

std::vector<std::vector<double>> DubinsPath::calc_lsr_from_origin(std::vector<double> e)
{
    double x_ = e[0] + sin(e[2]);
    double y_ = e[1] - 1 - cos(e[2]);
    double u1_square = pow(x_, 2) + pow(y_, 2);
    if (u1_square < 4)
    {
        return {};
    }
    double t1 = mod2pi(atan2(y_, x_));
    double u = sqrt(u1_square - 4);
    double theta = mod2pi(atan(2 / u));
    double t = mod2pi(t1 + theta);
    double v = mod2pi(t - e[2]);
    return {{'l', t}, {'s', u * _r}, {'r', v}};
}

std::vector<std::vector<double>> DubinsPath::calc_rsl_from_origin(std::vector<double> e)
{
    auto path = calc_lsr_from_origin(e);
    if (!path.empty())
    {
        path[0][0] = 'r';
        path[2][0] = 'l';
    }
    return path;
}

std::vector<std::vector<double>> DubinsPath::calc_lrl_from_origin(std::vector<double> e)
{
    double x_ = e[0] - sin(e[2]);
    double y_ = e[1] - 1 + cos(e[2]);
    double u1 = sqrt(pow(x_, 2) + pow(y_, 2));
    if (u1 > 4)
    {
        return {};
    }
    double t1 = atan2(y_, x_);
    double theta = acos(u1 / 4);
    double t = mod2pi(M_PI / 2 + t1 + theta);
    double u = mod2pi(M_PI + 2 * theta);
    double v = mod2pi(M_PI / 2 - t1 + theta + e[2]);
    return {{'l', t}, {'r', u}, {'l', v}};
}

std::vector<std::vector<double>> DubinsPath::calc_rlr_from_origin(std::vector<double> e)
{
    auto path = calc_lrl_from_origin(e);
    if (!path.empty())
    {
        path[0][0] = 'r';
        path[1][0] = 'l';
        path[2][0] = 'r';
    }
    return path;
}
// RETURNS X ,Y , Cost, SYMBOLIC PATHS
std::tuple<std::vector<double>, std::vector<double>, double, std::vector<std::vector<double>>> get_dubins_best_path_and_cost(
    std::vector<double> q_near, std::vector<double> q_rand, double _radius, double step)
{
    DubinsPath dubins_path(q_near, q_rand, _radius);
    // compute every possible path
    auto paths = dubins_path.calc_paths();
    // get the shortest + cost
    auto shortest_path_cost = dubins_path.get_shortest_path_cost();
    // discretize the best path
    Path full_path = gen_path(q_near, std::get<0>(shortest_path_cost), _radius, step);   
    std::tuple<std::vector<double>, std::vector<double>, double, std::vector<std::vector<double>>> best_path_and_cost;
    // set X
    std::get<0>(best_path_and_cost) = std::get<0>(full_path);
    // set Y
    std::get<1>(best_path_and_cost) = std::get<1>(full_path);
    // set cost
    std::get<2>(best_path_and_cost) = std::get<1>(shortest_path_cost);
    // set symbolic path
    std::get<3>(best_path_and_cost) = std::get<0>(shortest_path_cost);
    return best_path_and_cost;
}

void test_dubins()
{
  std::vector<double> q_near = {0, 0, 0};
  std::vector<double> q_rand = {-0.5, 0, 0};
  std::tuple<std::vector<double>, std::vector<double>, double, std::vector<std::vector<double>>> p =get_dubins_best_path_and_cost(q_near, q_rand, 1, 0.01);
  Path pa;
  std::get<0>(pa) = std::get<0>(p);
  std::get<1>(pa) = std::get<1>(p);
}