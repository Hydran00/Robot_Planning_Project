// #include <iostream>
// #include <cmath>
// #include <vector>
// #include <array>
// #include <algorithm>

// // #include "matplotlibcpp.h"  // You may need to install matplotlibcpp library for plotting

// // namespace plt = matplotlibcpp;

// class DubinsPath {
// public:
//     DubinsPath(const std::array<double, 3>& start, const std::array<double, 3>& end, double r = 1.0)
//         : _s(start), _e(end), _r(r) {}

//     // void drawPoint(const std::array<double, 3>& point, double arrow_length = 0.5) {
//     //     plt::plot(point[0], point[1], "o");
//     //     plt::arrow(point[0], point[1], arrow_length * cos(point[2]), arrow_length * sin(point[2]), "head_width=0.05");
//     // }

//     std::vector<std::vector<std::array<double, 2>>> genPath(const std::array<double, 3>& s, const std::vector<std::array<char, 2>>& path, double r = 1.0, bool section = true) {
//         auto calcTurnCenter = [r](const std::array<double, 3>& point, char dir) -> std::pair<double, double> {
//             double ang = (dir == 'l') ? (point[2] + M_PI / 2) : (point[2] - M_PI / 2);
//             double x = point[0] + cos(ang) * r;
//             double y = point[1] + sin(ang) * r;
//             return {x, y};
//         };

//         std::vector<double> r_x, r_y, ps_x, ps_y;
//         std::array<double, 3> start = s;
//         double yaw = s[2];

//         for (const auto& p : path) {
//             if (p[0] == 's') {
//                 for (double l = 0; l < p[1]; l += 0.5) {
//                     ps_x.push_back(start[0] + cos(yaw) * l);
//                     ps_y.push_back(start[1] + sin(yaw) * l);
//                 }
//                 ps_x.push_back(start[0] + cos(yaw) * p[1]);
//                 ps_y.push_back(start[1] + sin(yaw) * p[1]);
//                 if (section) {
//                     r_x.push_back(ps_x);
//                     r_y.push_back(ps_y);
//                 } else {
//                     r_x.insert(r_x.end(), ps_x.begin(), ps_x.end());
//                     r_y.insert(r_y.end(), ps_y.begin(), ps_y.end());
//                 }
//             } else {
//                 auto center = calcTurnCenter(start, p[0]);
//                 double ang_start = atan2(start[1] - center.second, start[0] - center.first);
//                 double ang_end = (p[0] == 'l') ? (ang_start + p[1]) : (ang_start - p[1]);
//                 double step = 0.5 / r;
//                 for (double ang = ang_start; (p[0] == 'l') ? (ang < ang_end) : (ang > ang_end); ang += (p[0] == 'l') ? step : -step) {
//                     ps_x.push_back(center.first + cos(ang) * r);
//                     ps_y.push_back(center.second + sin(ang) * r);
//                 }
//                 ps_x.push_back(center.first + cos(ang_end) * r);
//                 ps_y.push_back(center.second + sin(ang_end) * r);
//                 if (section) {
//                     r_x.push_back(ps_x);
//                     r_y.push_back(ps_y);
//                 } else {
//                     r_x.insert(r_x.end(), ps_x.begin(), ps_x.end());
//                     r_y.insert(r_y.end(), ps_y.begin(), ps_y.end());
//                 }
//                 yaw = (p[0] == 'l') ? (start[2] + p[1]) : (start[2] - p[1]);
//             }
//             start = {ps_x.back(), ps_y.back(), yaw};
//             ps_x.clear();
//             ps_y.clear();
//         }
//         return {r_x, r_y};
//     }

//     void test1() {
//         for (int i = 0; i < 360; i += 30) {
//             // plt::figure();
//             std::array<double, 3> start = {1, 1, 90 / 180.0 * M_PI};
//             std::array<double, 3> end = {3, 0, i / 180.0 * M_PI};
//             DubinsPath dubins(start, end, 4.0);
//             dubins.calcPaths();
//             auto paths = dubins.getShortestPath();
//             for (int j = 0; j < paths.size(); ++j) {
//                 // plt::subplot(2, 3, j + 1);
//                 // plt::title("{}{}{}", paths[j][0][0], paths[j][1][0], paths[j][2][0]);
//                 // drawPoint(start);
//                 // drawPoint(end);
//                 auto [xs, ys] = genPath(start, paths[j], 4.0);
//                 // for (int k = 0; k < 3; ++k) {
//                 //     plt::plot(xs[k], ys[k]);
//                 // }
//                 // plt::axis("equal");
//             }
//             // plt::show();
//         }
//     }

//     void test2() {
//         // plt::figure();
//         for (int i = 0; i < 12; ++i) {
//             std::array<double, 3> start = {1, 1, 300 / 180.0 * M_PI};
//             std::array<double, 3> end = {-2, 0, i * 30 / 180.0 * M_PI};
//             DubinsPath dubins(start, end, 4.0);
//             dubins.calcPaths();
//             auto [path, _] = dubins.getShortestPath();
//             // plt::subplot(3, 4, i + 1);
//             // plt::title("{}{}{}", path[0][0], path[1][0], path[2][0]);
//             // drawPoint(start);
//             // drawPoint(end);
//             auto [xs, ys] = genPath(start, path, 4.0);
//             // for (int j = 0; j < 3; ++j) {
//             //     plt::plot(xs[j], ys[j]);
//             // }
//             // plt::axis("equal");
//         }
//         // plt::show();
//     }

// private:
//     std::array<double, 3> _s;
//     std::array<double, 3> _e;
//     double _r;
//     std::vector<std::vector<std::array<double, 2>>> _paths;

//     std::vector<std::vector<std::array<char, 2>>> calcPaths() {
//         auto le = calcEnd();
//         std::vector<std::function<std::vector<std::array<char, 2>>(const std::array<double, 3>&)>> types = {
//             [this](const std::array<double, 3>& e) { return calcLSLFromOrigin(e); },
//             [this](const std::array<double, 3>& e) { return calcRSRFromOrigin(e); },
//             [this](const std::array<double, 3>& e) { return calcLSRFromOrigin(e); },
//             [this](const std::array<double, 3>& e) { return calcRSLFromOrigin(e); },
//             [this](const std::array<double, 3>& e) { return calcRLRFromOrigin(e); },
//             [this](const std::array<double, 3>& e) { return calcLRLFromOrigin(e); }
//         };
//         for (const auto& t : types) {
//             auto path = t(le);
//             if (!path.empty()) {
//                 _paths.push_back(path);
//             }
//         }
//         return _paths;
//     }

//     std::vector<std::array<char, 2>> getShortestPath() {
//         double shortestCost = std::numeric_limits<double>::infinity();
//         std::vector<std::array<char, 2>> shortestPath;
//         for (const auto& path : _paths) {
//             double cost = 0;
//             for (const auto& p : path) {
//                 cost += (p[0] == 's') ? p[1] : p[1] * _r;
//             }
//             if (cost < shortestCost) {
//                 shortestPath = path;
//                 shortestCost = cost;
//             }
//         }
//         return shortestPath;
//     }

//     std::array<double, 3> calcEnd() {
//         double ex = _e[0] - _s[0];
//         double ey = _e[1] - _s[1];
//         double lex = cos(_s[2]) * ex + sin(_s[2]) * ey;
//         double ley = -sin(_s[2]) * ex + cos(_s[2]) * ey;
//         double leyaw = _e[2] - _s[2];
//         lex = lex / _r;
//         ley = ley / _r;
//         return {lex, ley, leyaw};
//     }

//     double mod2pi(double theta) {
//         return theta - 2.0 * M_PI * floor(theta / (2.0 * M_PI));
//     }

//     std::vector<std::array<char, 2>> calcLSLFromOrigin(const std::array<double, 3>& e) {
//         double x_ = e[0] - sin(e[2]);
//         double y_ = e[1] - 1 + cos(e[2]);
//         double u = sqrt(x_ * x_ + y_ * y_);
//         double t = mod2pi(atan2(y_, x_));
//         double v = mod2pi(e[2] - t);
//         return {{'l', t}, {'s', u * _r}, {'l', v}};
//     }

//     std::vector<std::array<char, 2>> calcRSRFromOrigin(const std::array<double, 3>& e) {
//         auto e_ = e;
//         e_[1] = -e_[1];
//         e_[2] = mod2pi(-e_[2]);
//         auto path = calcLSLFromOrigin(e_);
//         path[0][0] = 'r';
//         path[2][0] = 'r';
//         return path;
//     }

//     std::vector<std::array<char, 2>> calcLSRFromOrigin(const std::array<double, 3>& e) {
//         double x_ = e[0] + sin(e[2]);
//         double y_ = e[1] - 1 - cos(e[2]);
//         double u1_square = x_ * x_ + y_ * y_;
//         if (u1_square < 4) {
//             return {};
//         }
//         double t1 = mod2pi(atan2(y_, x_));
//         double u = sqrt(u1_square - 4);
//         double theta = mod2pi(atan(2 / u));
//         double t = mod2pi(t1 + theta);
//         double v = mod2pi(t - e[2]);
//         return {{'l', t}, {'s', u * _r}, {'r', v}};
//     }

//     std::vector<std::array<char, 2>> calcRSLFromOrigin(const std::array<double, 3>& e) {
//         auto e_ = e;
//         e_[1] = -e_[1];
//         e_[2] = mod2pi(-e_[2]);
//         auto path = calcLSRFromOrigin(e_);
//         if (!path.empty()) {
//             path[0][0] = 'r';
//             path[2][0] = 'l';
//             return path;
//         } else {
//             return {};
//         }
//     }

//     std::vector<std::array<char, 2>> calcLRLFromOrigin(const std::array<double, 3>& e) {
//         double x_ = e[0] - sin(e[2]);
//         double y_ = e[1] - 1 + cos(e[2]);
//         double u1 = sqrt(x_ * x_ + y_ * y_);
//         if (u1 > 4) {
//             return {};
//         }
//         double t1 = atan2(y_, x_);
//         double theta = acos(u1 / 4);
//         double t = mod2pi(M_PI / 2 + t1 + theta);
//         double u = mod2pi(M_PI + 2 * theta);
//         double v = mod2pi(M_PI / 2 - t1 + theta + e[2]);
//         return {{'l', t}, {'r', u}, {'l', v}};
//     }

//     std::vector<std::array<char, 2>> calcRLRFromOrigin(const std::array<double, 3>& e) {
//         auto e_ = e;
//         e_[1] = -e_[1];
//         e_[2] = mod2pi(-e_[2]);
//         auto path = calcLRLFromOrigin(e_);
//         if (!path.empty()) {
//             path[0][0] = 'r';
//             path[1][0] = 'l';
//             path[2][0] = 'r';
//             return path;
//         } else {
//             return {};
//         }
//     }
// };

// int main() {
//     DubinsPath dubins;
//     dubins.test1();
//     return 0;
// }