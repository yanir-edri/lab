#include <iostream>
#include "common.cc"

namespace drone_exit_room {

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
        p2d center(const vp2d& pts) {
            ld x=0.0,z=0.0;
            for(auto p : pts) x += p.x, z += p.z;
            int n = int(pts.size());
            return {x/n,z/n};
        }
        //given a sector and the center of all points,
        //calculate a point representing the sector
        //(could be average, median or anything else).
        //farthest point will be selected as exit.
        //credits to the group of Peleg
        p2d func(vp2d& sec, p2d c) {
            sort(sec.begin(),sec.end(), [c](p2d a, p2d b) {
                return (a - c).abs() < (b - c).abs();
            });
            return sec[int(sec.size())/2];
            // int n = int(sec.size());
            // p2d avg{0.0,0.0};
            // for(auto p : sec) avg.x += p.x, avg.z += p.z;
            // return avg/n;
    //        return sec[n/2];
        }

        //Receives set of points, the size of each sector
        //and the offset of the first sector.
        //returns pair of exit point, distance.
        pair<p2d,ld> findExit(const vp2d& pts, int sa, int off) {
            p2d c = center(pts); //center of all points
            int sc = 360/sa; //number of sectors
            //create a vector of vectors, one for each sector
            //to hold its points.
            vector<vp2d> sec(sc);
            for(auto p : pts) {
                //calculate the sector of p by calculating
                //its angle from the center, minus the offset,
                //divided by the size of each sector.
                //we might reach a negative index. In this case,
                //we want to add sc (-1 -> last sector, -2 -> second last sector, etc.)
                int idx = int(((p-c).positive_angle()-off)/sa);
                if(idx<0) idx += sc;
                sec[idx].push_back(p);
            }
            //best point, best distance
            p2d bp = {0.0,0.0};
            ld bd = -1.0;
            for(int i = 0; i < sc; ++i) {
                auto& s = sec[i];
                if(int(s.size())<=1) continue;
                //calculate representing point's distance and update best point and distance accordingly
                p2d point = func(s,c);
                ld cd = (point-c).abs();
                if(cd-epsilon > bd) bd = cd, bp = point;
            }
            if(bd < -epsilon) return {{0.0,0.0},-1.0};
            return {bp,bd};
        }


        //Receives set of points and the size of each sector
        //returns pair of func(exit point section), farthest point in exit
        pair<p2d,ld> findExit(const vp2d& pts, int sa) {
            pair<p2d,ld> ans = {{0.0,0.0},-1.0},cur;
            //try all offsets and take the best one
            for(int i = 0; i < sa; ++i) {
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
                //if distance is negative, continue
                if(c.second<-epsilon) continue;
                //otherwise, this is a valid point
                ++valid, ans.push_back(c);
            }
            //if there are no valid points, return negative distance 
            //to indicate failure
            if(valid==0) return {{0.0,0.0},-1.0};
            //might want to process ans differently
            //we are simply taking the best
            pair<p2d,ld> best = ans[0];
            for(auto p : ans) if(p.second-epsilon > best.second) best = p;
            return best;
        }


    }

}