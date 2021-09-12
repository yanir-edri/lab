#include <bits/stdc++.h>
#define fin(i,s,n) for(auto i = s; i < n; ++i)
#define pb push_back
#define x first
#define y second
using namespace std;
using ld = long double;
const ld epsilon = 1e-6;
struct p2d {
    ld x, z;
    ld operator*(p2d o) { return (x * o.x + z * o.z); }
    p2d operator/(ld o) { return {x/o,z/o}; }
    p2d operator+(p2d o) { return {x + o.x, z + o.z}; }
    p2d operator-() { return {-x, -z}; }
    p2d operator-(p2d o) { return {x - o.x, z - o.z}; }
    ld abs() { return sqrtl(x * x + z * z); }
    ld angle() { return atan2l(z, x)*180.0/M_PI; }
    ld positive_angle() {
        ld a = angle();
        if(a < -epsilon) a += 360.0;
        return a;
    }
    ld dot_norm(p2d o) {
        return ((*this)*o)/(abs()*o.abs());
    }
    static p2d polar(ld theta) { return {cos(theta), sin(theta)}; }
};

struct p3d {
    ld x, y, z;
    explicit operator p2d() { return {x, z}; }
};
using vp2d = vector<p2d>;
using vp3d = vector<p3d>;
using vi = vector<int>;
using pld = pair<ld,ld>;
using vld = vector<ld>;
using vpld = vector<pld>;
struct line {
    ld m,b;
    p2d vec() { if(m==INFINITY) return {0.0,1.0}; return {1.0,m}; }
};
ostream& operator<<(ostream& os, line l) { return os << "(m=" << l.m <<",b="<<l.b<<")"; }
ostream &operator<<(ostream &os, p2d p) { return os << '{' << p.x << ',' << p.z << '}'; }
ostream &operator<<(ostream &os, p3d p) { return os << '{' << p.x << ',' << p.y << ',' << p.z << '}'; }
template<class A, class B> ostream& operator<<(ostream& os, pair<A,B> p) {
    return os << '{' << p.first << ',' << p.second << '}';
}
template<class T> ostream& operator<<(ostream& os, vector<T> v) {
    if(v.empty()) return os << "[]";
    os << '[' << v[0];
    fin(i,1,int(v.size())) os << ',' << v[i];
    return os << ']';
}

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
//combine csv_to_txt,txt_to_pts, return size
int csv_to_pts(const string& file, vp3d& pts) {
    int n = csv_to_txt(file);
    pts.resize(n);
    txt_to_pts(file,pts);
    return n;
}
namespace algo {
    line linear_fit(const vp2d& pts) {
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
    ld dist(line l, p2d p) {
        return abs((l.m*p.x) - p.z + l.b)/sqrtl((l.m*l.m) + 1);
    }
    ld global_best_dist = 1e9, global_best_angle = 0;
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
        ld bca = -1.0; int bi = -1;
        for(int i = 0; i < sc; ++i) {
            auto& s = sec[i];
            if(int(s.size())<=1) continue;
            line l = linear_fit(s);
            p2d lv = l.vec(), lv2 = -lv;
            //sector vector
            p2d sv = p2d::polar((i+1)*sa)-p2d::polar(i*sa);
            //cos(angle)
            ld ca = min(sv.dot_norm(lv),sv.dot_norm(lv2));
            if(bi==-1 || ca+epsilon<bca) bi = i, bca = ca;
            if(ca+epsilon < global_best_dist) {
                global_best_dist = ca;
                global_best_angle = i*sa+off;
            }
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
        cout << "findAngle("<<sa<<") -> " << as << endl;
        return as[n/2];
    }
    ld findAngle(const vp2d& pts) {
        vi sas{2,3,4,5,6,8,10,12,15};
        vpld ans;
        int valid = 0;
        for(int v : sas) {
            pld c = findAngle(pts,v);
            if(c.first>400.0) continue;
            cout << "findAngle("<<v<<") was valid and returned " << c << endl;
            ++valid, ans.pb(c);
        }
        if(valid==0) { cout << ":(((((((" << endl; exit(1); }
        cout << "global best angle is " << global_best_angle << endl;
        cout << "center is " << center(pts) << endl;
        ld fans = 0.0;
        for(auto p : ans ) fans += (p.x+p.y)/2.0;
        return fans/valid;
    }


}


int main() {

    vp3d pts3;
    int n = csv_to_pts("points",pts3);
    vp2d pts(n);
    for(int i = 0; i < n; ++i) pts[i] = p2d(pts3[i]);
    ld ang = algo::findAngle(pts);
    cout << "final angle is " << ang << endl;

    return 0;
}
