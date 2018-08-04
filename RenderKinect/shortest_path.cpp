#include "shortest_path.h"
#include <iostream>
#include <fstream>
#include <math.h>

#include <boost/algorithm/string.hpp>

namespace ShortestPath
{
    double eps = 1e-7;

    struct dwell
    {
        int xlabel;
        int ylabel;
        int xnode;
        int ynode;
        double overlap;
        double dist;
        
        dwell(int _xlabel, int _ylabel, int _xnode, int _ynode, double _overlap, double _dist)
        {
            xlabel = _xlabel;
            ylabel = _ylabel;
            xnode = _xnode;
            ynode = _ynode;
            overlap = _overlap;
            dist = _dist;
        }

        bool operator< (const dwell& that) const
        {
            return this->overlap > that.overlap || (fabs(this->overlap - that.overlap) < eps && this->dist < that.dist);
        }

        friend std::ostream& operator<< (std::ostream& out, const dwell& that)
        {
            out << "overlap " << that.xlabel << "[" << that.xnode << "] " << that.ylabel << "[" << that.ynode << "] = " << that.overlap;
            out << " distance = " << that.dist;
            return out;
        }
    };

    double euclidDistance(const Eigen::Vector3d& x, const Eigen::Vector3d& y)
    {
        Eigen::Vector3d z;
        z = x - y;
        return z.norm();
    }

    double arcDistance(const Eigen::Vector3d& x, const Eigen::Vector3d& y)
    {
        assert(fabs(x.norm() - 1.0) < 1e-7);
        assert(fabs(y.norm() - 1.0) < 1e-7);

        double costheta = x.dot(y);
        if (costheta < -1.0) costheta = -1.0;
        if (costheta > 1.0) costheta = 1.0;
        return acos(costheta);
    }

    std::vector<std::set<int>> init(const std::string& fileName)
    {
        std::ifstream poseKeypointResult("pose_keypoint_200_" + fileName + ".txt");
        std::vector<std::set<int>> visibleResult;
        std::string s;
        while (getline(poseKeypointResult, s))
        {
            std::set<int> set;
            getline(poseKeypointResult, s);
            std::vector<std::string> indices;
            boost::split(indices, s, boost::is_any_of(" "));
            for (const std::string& id: indices)
            {
                if (id.size() > 0)
                {
                    set.insert(std::stoi(id));
                }
            }
            visibleResult.push_back(set);
        }
        poseKeypointResult.close();
        return visibleResult;
    }

    double calcOverlap(const std::set<int>& A, const std::set<int>& B)
    {
        std::vector<int> i, u;
        std::set_intersection(A.begin(), A.end(), B.begin(), B.end(), std::back_inserter(i));
        std::set_union(A.begin(), A.end(), B.begin(), B.end(), std::back_inserter(u));
        return (double)i.size() / (double)u.size();
    }

    std::vector<Eigen::Vector3d> getView(const Eigen::Vector3d& camera_normal, int horizontal_count, int vertical_count)
    {
        std::vector<Eigen::Vector3d> views;

        double horizontal_angle = 2 * M_PI / horizontal_count;
        double vertical_angle = M_PI / (vertical_count - 1);

        for (int i = 0; i < vertical_count; ++i)
        {
            for (int j = 0; j < horizontal_count; ++j)
            {
                if ((i == 0 || i == vertical_count - 1) && j)
                {
                    continue;
                }

                double horizontal_spin = horizontal_angle * j;
                double vertical_spin = vertical_angle * i;

                Eigen::Matrix3d rotation, rotation_inv;
                rotation = Eigen::AngleAxisd(vertical_spin, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(horizontal_spin, camera_normal);
                rotation_inv = rotation.inverse();
                views.push_back(rotation_inv * camera_normal);
            }
        }
        return views;
    }

    void execute(const std::string& fileName)
    {
        std::cout << "name = " << fileName << std::endl;
        std::vector<std::set<int>> group = init(fileName);
        std::vector<Eigen::Vector3d> views = getView(Eigen::Vector3d(0, 0, -1), 18, 13);
        
        std::vector<int> selected_views = {31, 23, 84, 57, 94, 148, 179, 171, 50, 35, 26};


        int ecnt = 0;
        std::vector<dwell> dummy;
        
        int n = selected_views.size();
        double** g = new double*[n];
        for (int i = 0; i < n; ++i)
        {
            g[i] = new double[n];
        }
        double** f = new double*[1 << n];
        for (int i = 0; i < (1 << n); ++i)
        {
            f[i] = new double[n];
            for (int j = 0; j < n; ++j)
            {
                f[i][j] = 1e10;
            }
        }
        int** fa = new int*[1 << n];
        for (int i = 0; i < (1 << n); ++i)
        {
            fa[i] = new int[n];
            for (int j = 0; j < n; ++j)
            {
                fa[i][j] = -1;
            }
        }

        for (int i = 0; i < selected_views.size(); ++i)
        {
            g[i][i] = 0.0;
            for (int j = i + 1; j < selected_views.size(); ++j)
            {
                int x = selected_views[i];
                int y = selected_views[j];
                double overlap = calcOverlap(group[x], group[y]);
                double dist = arcDistance(views[x], views[y]);
                g[i][j] = g[j][i] = dist;
                /* if (overlap + eps >= 0.5 || dist <= 2.0 + eps)
                {
                    ++ecnt;
                    std::cout << "overlap " << i << "[" << x << "] " << j << "[" << y << "] = ";
                    std::cout << calcOverlap(group[x], group[y]) << "(" << group[x].size() << ", " << group[y].size() << ")";
                    std::cout << " distance = " << arcDistance(views[x], views[y]);
                    std::cout << std::endl;
                } */
                dummy.push_back(dwell(i, j, x, y, overlap, dist));
            }
        }
        std::sort(dummy.begin(), dummy.end());
        for (const dwell& dd: dummy)
        {
            std::cout << dd << std::endl;
        }

        // std::cout << "edge count = " << ecnt << std::endl;
        
        /*for (int selected_view: selected_views)
        {
            Eigen::Vector3d vv = views[selected_view];
            std::cout << "view " << selected_view << ": [" << vv[0] << ", " << vv[1] << ", " << vv[2] << "]" << std::endl;
        }*/

        // TSP

        for (int i = 0; i < n; ++i)
        {
            f[0][i] = 0.0;
        }
        for (int i = 1; i < (1 << n); ++i)
        {
            for (int v = 0; v < n; ++v)
            {
                if (i & (1 << v))
                {
                    // f[i][v] = ...
                    for (int u = 0; u < n; ++u)
                    {
                        double dist = f[i - (1 << v)][u] + g[u][v];
                        if (dist < f[i][v])
                        {
                            f[i][v] = dist;
                            fa[i][v] = u;
                        }
                    }
                }
            }
        }

        std::vector<int> answer;
        double mindist = 1e10;
        int where = -1;
        for (int i = 0; i < n; ++i)
        {
            if (f[(1 << n) - 1][i] < mindist)
            {
                mindist = f[(1 << n) - 1][i];
                where = i;
            }
        }

        int curbin = (1 << n) - 1;
        int where0 = where;
        while (curbin)
        {
            answer.push_back(where0);
            int nxt = fa[curbin][where0];
            curbin -= (1 << where0);
            where0 = nxt;
        }

        std::cout << "minimum distance = " << f[(1 << n) - 1][where] << std::endl;
        for (int i = 0; i < answer.size() - 1; ++i)
        {
            int x = selected_views[answer[i]];
            int y = selected_views[answer[i + 1]];
            std::cout << answer[i] << "(" << x << ") " << answer[i + 1] << "(" << y << ")";
            std::cout << " dis = " << g[answer[i]][answer[i + 1]] << " overlap = " << calcOverlap(group[x], group[y]);
            std::cout << std::endl;
        }

    }
}