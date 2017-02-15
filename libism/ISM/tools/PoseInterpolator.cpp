/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "PoseInterpolator.hpp"
#include "../utility/TableHelper.hpp"
#include "../utility/GeometryHelper.hpp"

#include <fstream>
#include <set>

namespace ISM
{
void PoseInterpolator::interpolate(const std::string& sourceFile, const std::string& targetFile, int stepNumber)
{
    if (stepNumber < 1)
    {
        std::cerr << "stepNumber is < 1!" << std::endl;
        exit(0);
    }

    // copy source to target, so that only the copy will be modified
    std::ifstream source(sourceFile, std::fstream::binary);
    std::ofstream target(targetFile, std::fstream::trunc|std::fstream::binary);
    target << source.rdbuf();

    TableHelper sourceHelper(sourceFile);
    TableHelper targetHelper(targetFile);
    // drop tables to write the sets in the right order
    targetHelper.dropTable("recorded_objects");
    targetHelper.dropTable("recorded_sets");
    targetHelper.createTablesIfNecessary();

    // assume we have only one pattern (scene)
    std::string pattern = sourceHelper.getRecordedPatternNames().at(0);
    // counter begins with 1 because databases IDs begin also with 1
    int counter = 1;
    ObjectSetPtr from = sourceHelper.getRecordedObjectSet(counter);
    ObjectSetPtr to = sourceHelper.getRecordedObjectSet(counter + 1);
    while (from->objects.size() != 0 && to->objects.size() != 0)
    {
        std::cout << "Interpolating between object sets " << counter << " and " << counter + 1 << "..." << std::endl;
        targetHelper.insertRecordedObjectSet(from, pattern);
        for (ObjectSetPtr objectSet : interpolateSets(from, to, stepNumber))
        {
            targetHelper.insertRecordedObjectSet(objectSet, pattern);
        }
        ++counter;
        from = sourceHelper.getRecordedObjectSet(counter);
        to = sourceHelper.getRecordedObjectSet(counter + 1);
    }
    // insert the last ObjectSet
    targetHelper.insertRecordedObjectSet(from, pattern);
    std::cout << "Finished!" << std::endl;
}

std::vector<PosePtr> PoseInterpolator::interpolatePoses(PosePtr from, PosePtr to, int stepNumber)
{
    std::vector<PosePtr> result;
    int divisor = stepNumber + 1;
    Eigen::Vector3d fromPosition = GeometryHelper::pointToVector(from->point);
    Eigen::Vector3d toPosition = GeometryHelper::pointToVector(to->point);
    Eigen::Quaterniond fromOrient = GeometryHelper::quatToEigenQuat(from->quat);
    Eigen::Quaterniond toOrient = GeometryHelper::quatToEigenQuat(to->quat);
    Eigen::Vector3d interpolatedPosition;
    Eigen::Quaterniond interpolatedQuat;
    for (int i = 1; i <= stepNumber; i++)
    {
        interpolatedPosition = fromPosition * (divisor - i)/divisor + toPosition * i/divisor;
        interpolatedQuat = fromOrient.slerp((double)i/divisor, toOrient);
        result.push_back(PosePtr(new Pose(GeometryHelper::vectorToPoint(interpolatedPosition), GeometryHelper::eigenQuatToQuat(interpolatedQuat))));
    }

    return result;
}

std::vector<ObjectSetPtr> PoseInterpolator::interpolateSets(const ObjectSetPtr from, const ObjectSetPtr to, const int stepNumber)
{
    std::vector<ObjectSetPtr> result;
    // prepare sets in the vector
    for (int i = 0; i < stepNumber; i++)
    {
        result.push_back(ObjectSetPtr(new ObjectSet()));
    }
    std::vector<PosePtr> interpolatedPoses;
    PosePtr pose;
    ObjectPtr interpolatedObject;
    for (unsigned i = 0; i < from->objects.size(); i++)
    {
        interpolatedPoses = interpolatePoses(from->objects.at(i)->pose, to->objects.at(i)->pose, stepNumber);
        for (unsigned j = 0; j < interpolatedPoses.size(); j++)
        {
            pose = interpolatedPoses.at(j);
            interpolatedObject = ObjectPtr(new Object(*(from->objects.at(i).get()))); // copy objectPtr
            interpolatedObject->pose = pose;
            result.at(j)->insert(interpolatedObject);
        }
    }

    return result;
}

}
