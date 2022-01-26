#include <descriptor.hpp>

namespace descriptor{
// The accumulated lens could be simplified by using the already available edge lengths.
// We implemented this way to follow the original code.
PointVector samplePoints(PointVector points, double step){
    std::vector<PointVector> lineVectors;
    std::vector<double> lineLens;
    for(size_t i = 0; i < points.size(); ++i){
        size_t modI = (i+1) % points.size();
        vecPtT lineStart = {points[i][0], points[i][1]};
        vecPtT lineEnd = {points[modI][0], points[modI][1]};
        PointVector line = {lineStart, lineEnd};
        lineVectors.push_back(line);
        lineLens.push_back(euclideanDistance2D(lineStart, lineEnd));
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
        // define how much of the perimeter we should move
        if(currLine != 0){
            d = (std::abs(coveredPerim - normalizedAccumLens[currLine-1])/
                std::abs(normalizedAccumLens[currLine]-normalizedAccumLens[currLine-1]));
        } else if(coveredPerim > 0){
            d = coveredPerim/normalizedAccumLens[currLine];
        }

        // get point relative to current % of line segment
        PointVector l = lineVectors[currLine];
        vecPtT a = l[0];
        vecPtT b = l[1];
        double px = a[0] + d * (b[0] - a[0]);
        double py = a[1] + d * (b[1] - a[1]);
        vecPtT pt = {px, py};
        sampledPoints.push_back(pt);
        coveredPerim += step;
    }

    return sampledPoints;
}

std::vector<double> centroidDist(PointVector points, PointVector sampledPoints){
    vecPtT centroid(2, 0);
    for(vecPtT p : points){
        centroid[0] += p[0];
        centroid[1] += p[1];
    }
    centroid[0] /= points.size();
    centroid[1] /= points.size();

    std::vector<double> descriptor;
    descriptor.reserve(sampledPoints.size());
    for(vecPtT sp : sampledPoints){
        descriptor.push_back(euclideanDistance2D(sp, centroid));
    }
    return descriptor;
}

// based on the official OpenCV tutorial: https://docs.opencv.org/3.4/d8/d01/tutorial_discrete_fourier_transform.html
std::vector<double> invariantFourier(std::vector<double> centroidDesc){
    std::vector<double> dftDesc;
    cv::Mat matDesc(centroidDesc);
    cv::Mat planes[] = {matDesc,
                        cv::Mat::zeros(cv::Size(1, centroidDesc.size()), CV_64FC1)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
    cv::dft(complexI, complexI);            // this way the result may fit in the source matrix
    // compute the magnitude
    cv::split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    cv::magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    cv::Mat magI = planes[0];

    if (magI.isContinuous()) {
        dftDesc.assign((double*)magI.data, (double*)magI.data + magI.total()*magI.channels());
    } else {
        for (int i = 0; i < magI.rows; ++i) {
            dftDesc.insert(
                dftDesc.end(),
                magI.ptr<double>(i), magI.ptr<double>(i)+magI.cols*magI.channels());
        }
    }
    return dftDesc;
}

std::vector<double> compute(PointVector points){
    PointVector sampledPoints = samplePoints(points, 0.05);
    std::vector<double> centrDesc = centroidDist(points, sampledPoints);
    return invariantFourier(centrDesc);
}
}