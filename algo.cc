#include <iostream>
#include "common.cc"

//converts csv file to txt file, returns number of points
int csv_to_txt(const string &file) {
    ifstream rd(file + ".csv");
    ofstream wr(file + ".txt");
    string s;
    int n = 0;
    while (getline(rd,s)) {
        ++n;
        replace(s.begin(), s.end(), ',', ' ');
        wr << s << '\n';
    }
    wr << '\n';
    rd.close(), wr.close();
    return n;
}

//fills vector with points from txt file
void txt_to_pts(const string &file, vp3d& pts) {
    ifstream rd(file + ".txt");
    for(auto& p : pts) rd >> p.x >> p.y >> p.z;
    rd.close();
}

//fills vector with points from txt file
void txt_to_pts_v2(const string &file, vp3d& pts) {
    ifstream rd(file + ".txt");
    ld d1, d2, d3;
    for(auto& p : pts) rd >> p.x >> p.y >> p.z >> d1 >> d2 >> d3;
    rd.close();
}
//combine csv_to_txt,txt_to_pts, return size
int csv_to_pts(const string& file, vp3d& pts) {
    int n = csv_to_txt(file);
    pts.resize(n);
//    txt_to_pts(file,pts);
    txt_to_pts_v2(file,pts);
    return n;
}
#define DBG_ALGO 0
//credits to the group of Peleg
namespace algo {
    Line linear_fit(const vp2d& pts) {
        //assume size > 1
        {
            //check if slope is inf
            ld x0 = pts[0].x,e = 0.0;
            for(auto p : pts) e += abs(p.x-x0);
            if(e < epsilon) return {INFINITY, x0};
        }
        int n = int(pts.size());
        ld sumX, sumZ, sumXZ, sumX2;
        sumX = sumZ = sumXZ = sumX2 = 0.0;
        for(auto p : pts) {
            sumX += p.x;
            sumZ += p.z;
            sumXZ += p.x * p.z;
            sumX2 += p.x * p.x;
        }
        ld m = ((n*sumXZ) - (sumX*sumZ)) / ((n*sumX2) - (sumX*sumX));
        ld b = (sumZ - (m*sumX)) / n;
        return {m,b};
    }
    p2d center(const vp2d& pts) {
        ld x=0.0,z=0.0;
        for(auto p : pts) x += p.x, z += p.z;
        int n = int(pts.size());
        return {x/n,z/n};
    }
    ld dist(Line l, p2d p) {
        return abs((l.m*p.x) - p.z + l.b)/sqrtl((l.m*l.m) + 1);
    }
    p2d func(const vp2d& sec) {
        int n = int(sec.size());
        p2d avg{0.0,0.0};
        for(auto p : sec) avg.x += p.x, avg.z += p.z;
        return avg/n;
//        return sec[n/2];
    }
    ld global_best_dist = -1, global_best_angle = 0;
    pld findAngle(const vp2d& pts, int sa, int off) {
        p2d c = center(pts);
        int sc = 360/sa;
        vector<vp2d> sec(sc);
        for(auto p : pts) {
            int idx = int(((p-c).positive_angle()-off)/sa);
            if(idx<0) idx += sc;
            sec[idx].push_back(p);
        }
        //b for best
        ld bd = -1.0; int bi = -1;
        for(int i = 0; i < sc; ++i) {
            auto& s = sec[i];
            if(int(s.size())<=1) continue;
            sort(s.begin(),s.end(),[c](p2d a, p2d b) {
                return (a-c).abs() < (b-c).abs();
            });

            p2d point = func(s);
            ld cd = (point-c).abs();
            if(cd-epsilon > bd) bd = cd, bi = i;
            if(cd-epsilon > global_best_dist) global_best_dist=cd,global_best_angle = (point-c).positive_angle();
        }
        if(bi==-1) return {500.0,500.0};
        return {bi*sa+off,(bi+1)*sa+off};
    }
    pld findAngle(const vp2d& pts, int sa) {
        vpld as;
        fin(i,0,sa) as.pb(findAngle(pts,sa,i));
        sort(as.begin(),as.end());
        while(!as.empty()&&as.back().first>400.0) as.pop_back();
         if(as.empty()) return {500.0,500.0};
        int n = int(as.size());
        //cout << "findAngle("<<sa<<") -> " << as << endl;
        return as[n/2];
    }
    ld findAngle(const vp2d& pts) {
        vi sas{2,3,4,5,6,8,10,12,15};
        vpld ans;
        int valid = 0;
        for(int v : sas) {
            pld c = findAngle(pts,v);
            if(c.first>400.0) continue;
            //cout << "findAngle("<<v<<") was valid and returned " << c << endl;
            ++valid, ans.pb(c);
        }
        if(valid==0) { cout << ":(((((((" << endl; exit(1); }
        //cout << "global best angle is " << global_best_angle << endl;
        //cout << "center is " << center(pts) << endl;
        {
            p2d p = pts[0], c = center(pts);
            for(auto P : pts) if((P-c).abs()-epsilon>(p-c).abs()) p = P;
            //cout << "farthest point is " << p << ", angle is " << p.positive_angle() << endl;
        }
        ld fans = 0.0;
        for(auto p : ans ) fans += (p.first+p.second)/2.0;
        return fans/valid;
    }


}
/*
	closest point on line
	ld closestX, closestZ;
	closestX = (m[closestLine]*center[2] + center[0] - m[closestLine]*b[closestLine])/((m[closestLine]*m[closestLine]) + 1);
	closestZ = (m[closestLine]*center[0] + m[closestLine]*m[closestLine]*center[2] + b[closestLine])/((m[closestLine]*m[closestLine]) + 1);
*/
/*
int main() {

    vp3d pts3;
    int n = csv_to_pts("pointData0",pts3);
//    int n = csv_to_pts("points",pts3);
    vp2d pts(n);
    for(int i = 0; i < n; ++i) pts[i] = p2d(pts3[i]);
    ld ang = algo::findAngle(pts);
    cout << "final angle is " << ang << endl;

    return 0;
}
*/
