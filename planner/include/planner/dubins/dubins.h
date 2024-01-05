#ifndef DUBINS_PATH_H
#define DUBINS_PATH_H

#include <cmath>
#include <vector>
#include <tuple>

class DubinsPath
{
private:
    std::vector<std::vector<double>> _s;
    std::vector<std::vector<double>> _e;
    double _r;
    std::vector<std::vector<std::vector<double>>> _paths;

public:
    DubinsPath(std::vector<double> start, std::vector<double> end, double r = 1.0);

    std::vector<std::vector<std::vector<double>>> calc_paths();
    std::vector<std::vector<double>> get_shortest_path();
    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> gen_path(
        const std::vector<double> &s, const std::vector<std::vector<double>> &path, double r, bool section);

private:
    std::vector<double> calc_end();
    double mod2pi(double theta);
    std::vector<std::vector<double>> calc_lsl_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_rsr_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_lsr_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_rsl_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_lrl_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_rlr_from_origin(std::vector<double> e);
};

#endif // DUBINS_PATH_H