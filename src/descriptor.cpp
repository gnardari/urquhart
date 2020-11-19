#include <descriptor.hpp>

namespace descriptor{
PointVector samplePoints(PointVector points, double step){
    std::vector<PointVector> lineVectors;
    std::vector<double> lineLens;
    for(size_t i = 0; i < points.size(); ++i){
        size_t modI = (i+1) % points.size();
        PointT lineStart = {points[i][0], points[i][1]};
        PointT lineEnd = {points[modI][0], points[modI][1]};
        PointVector line = {lineStart, lineEnd};
        lineVectors.push_back(line);
        lineLens.push_back(euclideanDistance(lineStart, lineEnd));
    }
    std::vector<double> normalizedAccumLens;
    double totalLen = std::accumulate(lineLens.begin(), lineLens.end(), 0.0);
    for(double len : lineLens){
        if(normalizedAccumLens.size() == 0)
            normalizedAccumLens.push_back(len/totalLen);
        else {
            double currSum = normalizedAccumLens[normalizedAccumLens.size()-1];
            normalizedAccumLens.push_back((len/totalLen)+currSum);
        }
    } 

    size_t currLine = 0;
    double coveredPerim = 0;
    PointVector sampledPoints;
    double d = 0;
    while(std::abs(coveredPerim - 1.0) > 0.0001){
        if(coveredPerim > normalizedAccumLens[currLine]){
            currLine++;
            continue;
        }
        if(currLine != 0){
            d = (std::abs(coveredPerim - normalizedAccumLens[currLine-1])/
                std::abs(normalizedAccumLens[currLine]-normalizedAccumLens[currLine-1]));
        } else if(coveredPerim > 0){
            d = coveredPerim/normalizedAccumLens[currLine];
        }

        // get point relative to current % of line segment
        PointVector l = lineVectors[currLine];
        PointT a = l[0];
        PointT b = l[1];
        double px = a[0] + d * (b[0] - a[0]);
        double py = a[1] + d * (b[1] - a[1]);
        PointT pt = {px, py};
        sampledPoints.push_back(pt);
        coveredPerim += step;
    }

    return sampledPoints;
}

std::vector<double> centroidDist(PointVector points, PointVector sampledPoints){
    PointT centroid(2, 0);
    for(PointT p : points){
        centroid[0] += p[0];
        centroid[1] += p[1];
    }
    centroid[0] /= points.size();
    centroid[1] /= points.size();

    std::vector<double> descriptor;
    descriptor.reserve(sampledPoints.size());
    for(PointT sp : sampledPoints){
        descriptor.push_back(euclideanDistance(sp, centroid));
    }
    return descriptor;
}
std::vector<double> invariantFourier(std::vector<double> centroidDesc){
    std::vector<double> dftDesc;
    cv::dft(centroidDesc, dftDesc);
    for(auto& e : dftDesc){
        e = std::abs(e);
    }
    return dftDesc;
}

std::vector<double> compute(PointVector points){
    PointVector sampledPoints = samplePoints(points, 0.05);
    std::vector<double> centrDesc = centroidDist(points, sampledPoints);
    auto inv = invariantFourier(centrDesc);

    return inv;
}
}