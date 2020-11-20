#include <matching.hpp>
#include <observation.hpp>
#include <utils.hpp>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

PointVector readFile(string path){
    PointVector landmarks;
    ifstream infile(path);

    while (infile)
    {
        string s;
        if (!getline(infile, s)) break;

        istringstream ss(s);
        PointT record;

        while (ss){
        string s;
        if (!getline(ss, s, ',')) break;
        record.push_back(stod(s));
        }

        landmarks.push_back(record);
    }
    if (!infile.eof())
    {
        cerr << "Fooey!\n";
    }
    return landmarks;
}

int main(){
    // PointVector points{{0,0}, {0,1.1}, {1,0}, {1,1}, {1,2}};
    PointVector points = readFile("/media/gnardari/DATA/observations/obs4.out");
    urquhart::Observation obs(points);

    // PointVector pointr{{0,0}, {-1.1,0}, {0,1}, {-1,1}, {-2,1}};
    PointVector pointsr = readFile("/media/gnardari/DATA/observations/obs5.out");
    urquhart::Observation obsr(pointsr);

    auto matches = matching::hierarchyMatching(obs, obsr, 5);
    for(auto m : matches){
        std::cout << m.first[0] << "," << m.first[1] << ",";
        std::cout << m.second[0] << "," << m.second[1] << std::endl;
    }
    return 0;
}