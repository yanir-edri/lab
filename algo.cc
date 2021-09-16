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
    txt_to_pts(file,pts);
//    txt_to_pts_v2(file,pts);
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
    p2d func(vp2d& sec, p2d c) {
        sort(sec.begin(),sec.end(), [c](p2d a, p2d b) {
            return (a - c).abs() < (b - c).abs();
        });
        return sec[int(sec.size())/2];
        int n = int(sec.size());
        p2d avg{0.0,0.0};
        for(auto p : sec) avg.x += p.x, avg.z += p.z;
        return avg/n;
//        return sec[n/2];
    }

    //Receives set of points, the size of each sector
    //and the offset of the first sector.
    //returns pair of exit point, distance.
    pair<p2d,ld> findExit(const vp2d& pts, int sa, int off) {
        p2d c = center(pts);
        int sc = 360/sa;
        vector<vp2d> sec(sc);
        for(auto p : pts) {
            int idx = int(((p-c).positive_angle()-off)/sa);
            if(idx<0) idx += sc;
            sec[idx].push_back(p);
        }
        //b for best
        p2d bp = {0.0,0.0};
        ld bd = -1.0; int bi = -1;
        for(int i = 0; i < sc; ++i) {
            auto& s = sec[i];
            if(int(s.size())<=1) continue;
            p2d point = func(s,c);
            ld cd = (point-c).abs();
            if(cd-epsilon > bd) bd = cd, bi = i, bp = point;
        }
        if(bi==-1) return {{0.0,0.0},-1.0};
        return {bp,bd};
    }


    //Receives set of points and the size of each sector
    //returns pair of func(exit point section), farthest point in exit
    pair<p2d,ld> findExit(const vp2d& pts, int sa) {
        pair<p2d,ld> ans = {{0.0,0.0},-1.0},cur;
        //try all offsets and take the best one
        fin(i,0,sa) {
            cur = findExit(pts,sa,i);
            if(cur.second-epsilon > ans.second) ans = cur;
        }
        if(cur.second<-epsilon) return {{0.0,0.0},-1.0}; 
        return ans;
    }
    //return a pair of exit point and distance
    //if returned distance is negative, no exit point was found
    pair<p2d,ld> findExit(const vp2d& pts) {
        //list of sector angles we want to try
        //must divide 360
        vi sas{2,3,4,5,6,8,10,12,15};
        vector<pair<p2d,ld>> ans;
        int valid = 0;
        for(int v : sas) {
            pair<p2d,ld> c = findExit(pts,v);
            if(c.second<-epsilon) continue;
//            cout  << "findExit("<<v<<") was valid and returned " << c << endl;
//            cout << "which has angle " << c.first.positive_angle() << endl;
            ++valid, ans.pb(c);
        }
        if(valid==0) return {{0.0,0.0},-1.0};
        //cout << "global best angle is " << global_best_angle << endl;
        //cout << "center is " << center(pts) << endl;
        //might want to process ans differently
        //sort(ans.begin(),ans.end(),[]())
        pair<p2d,ld> best = ans[0];
        for(auto p : ans) if(p.second-epsilon > best.second) best = p;
        return best;
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
    ld ang = algo::findExit(pts);
    cout << "final angle is " << ang << endl;

    return 0;
}
*/