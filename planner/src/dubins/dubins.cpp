#include "planner/dubins/dubins.h"
#include <iostream>

DubinsPath::DubinsPath(std::vector<double> start, std::vector<double> end, double r)
    : _s({start}), _e({end}), _r(r) {}

std::vector<std::vector<std::vector<double>>> DubinsPath::calc_paths() {
    auto le = calc_end();
    auto types = {&DubinsPath::calc_lsl_from_origin,
                  &DubinsPath::calc_rsr_from_origin,
                  &DubinsPath::calc_lsr_from_origin,
                  &DubinsPath::calc_rsl_from_origin,
                  &DubinsPath::calc_rlr_from_origin,
                  &DubinsPath::calc_lrl_from_origin};
    for (auto t : types) {
        auto path = (this->*t)(le);
        if (!path.empty()) {
            _paths.push_back(path);
        }
    }
    return _paths;
}

std::vector<std::vector<double>> DubinsPath::get_shortest_path() {
    double shortest_cost = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> shortest_path;
    for (auto &path : _paths) {
        double cost = 0;
        for (auto &p : path) {
            cost += (p[0] == 's') ? p[1] : p[1] * _r;
        }
        if (cost < shortest_cost) {
            shortest_path = path;
            shortest_cost = cost;
        }
    }
    return shortest_path;
}

std::vector<double> DubinsPath::calc_end() {
    double ex = _e[0][0] - _s[0][0];
    double ey = _e[0][1] - _s[0][1];

    double lex = cos(_s[0][2]) * ex + sin(_s[0][2]) * ey;
    double ley = -sin(_s[0][2]) * ex + cos(_s[0][2]) * ey;
    double leyaw = _e[0][2] - _s[0][2];
    lex = lex / _r;
    ley = ley / _r;
    return {lex, ley, leyaw};
}

double DubinsPath::mod2pi(double theta) {
    return theta - 2.0 * M_PI * floor(theta / (2.0 * M_PI));
}

std::vector<std::vector<double>> DubinsPath::calc_lsl_from_origin(std::vector<double> e) {
    double x_ = e[0] - sin(e[2]);
    double y_ = e[1] - 1 + cos(e[2]);

    double u = sqrt(pow(x_, 2) + pow(y_, 2));
    double t = mod2pi(atan2(y_, x_));
    double v = mod2pi(e[2] - t);
    return {{'l', t}, {'s', u * _r}, {'l', v}};
}

std::vector<std::vector<double>> DubinsPath::calc_rsr_from_origin(std::vector<double> e) {
    auto path = calc_lsl_from_origin(e);
    path[0][0] = 'r';
    path[2][0] = 'r';
    return path;
}

std::vector<std::vector<double>> DubinsPath::calc_lsr_from_origin(std::vector<double> e) {
    double x_ = e[0] + sin(e[2]);
    double y_ = e[1] - 1 - cos(e[2]);
    double u1_square = pow(x_, 2) + pow(y_, 2);
    if (u1_square < 4) {
        return {};
    }
    double t1 = mod2pi(atan2(y_, x_));
    double u = sqrt(u1_square - 4);
    double theta = mod2pi(atan(2 / u));
    double t = mod2pi(t1 + theta);
    double v = mod2pi(t - e[2]);
    return {{'l', t}, {'s', u * _r}, {'r', v}};
}

std::vector<std::vector<double>> DubinsPath::calc_rsl_from_origin(std::vector<double> e) {
    auto path = calc_lsr_from_origin(e);
    if (!path.empty()) {
        path[0][0] = 'r';
        path[2][0] = 'l';
    }
    return path;
}

std::vector<std::vector<double>> DubinsPath::calc_lrl_from_origin(std::vector<double> e) {
    double x_ = e[0] - sin(e[2]);
    double y_ = e[1] - 1 + cos(e[2]);
    double u1 = sqrt(pow(x_, 2) + pow(y_, 2));
    if (u1 > 4) {
        return {};
    }
    double t1 = atan2(y_, x_);
    double theta = acos(u1 / 4);
    double t = mod2pi(M_PI / 2 + t1 + theta);
    double u = mod2pi(M_PI + 2 * theta);
    double v = mod2pi(M_PI / 2 - t1 + theta + e[2]);
    return {{'l', t}, {'r', u}, {'l', v}};
}

std::vector<std::vector<double>> DubinsPath::calc_rlr_from_origin(std::vector<double> e) {
    auto path = calc_lrl_from_origin(e);
    if (!path.empty()) {
        path[0][0] = 'r';
        path[1][0] = 'l';
        path[2][0] = 'r';
    }
    return path;
}

int main()
{
    std::vector<double> start = {0.0, 0.0, 0.0};
    std::vector<double> end = {5.0, 5.0, M_PI};
    DubinsPath dubins_path(start, end);
    auto paths = dubins_path.calc_paths();
    auto shortest_path = dubins_path.get_shortest_path();
    std::cout << "shortest path: " << shortest_path.size() << std::endl;
    // Print or use the paths as needed
    return 0;
}