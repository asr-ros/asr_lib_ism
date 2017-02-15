/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "RotationInvariantObjectsRotator.hpp"
#include "../utility/TableHelper.hpp"
#include "../utility/GeometryHelper.hpp"

#include <fstream>
#include <set>

namespace ISM
{

void RotationInvariantObjectsRotator::rotateRotationInvariantObjects(const std::string& sourceFile, const std::string& targetFile, const std::string& objectType, const std::string& objectId, bool useMapFrame, const std::vector<std::string>& rotationInvariantTypes)
{
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
    ObjectSetPtr currentSet = sourceHelper.getRecordedObjectSet(counter);
    while (currentSet->objects.size() != 0)
    {
        std::cout << "Processing object set " << counter << "..." << std::endl;
        if (!useMapFrame)
        {
            normalizeRotationInvarienceObjects(currentSet, getReferenceObject(currentSet, objectType, objectId)->pose->quat->eigen, rotationInvariantTypes);
        }
        else
        {
            normalizeRotationInvarienceObjects(currentSet, Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), rotationInvariantTypes);
        }
        targetHelper.insertRecordedObjectSet(currentSet, pattern);
        ++counter;
        currentSet = sourceHelper.getRecordedObjectSet(counter);
    }
}

Eigen::AngleAxisd RotationInvariantObjectsRotator::calculateTransform(const Eigen::Vector3d& objectAxis, const Eigen::Vector3d& referenceAxis, const Eigen::Vector3d& unitAxis)
{
    double angle = ISM::GeometryHelper::getAngleBetweenAxes(referenceAxis, objectAxis);
    Eigen::Vector3d rotationAxis = objectAxis.cross(referenceAxis);
    rotationAxis.normalize();
    angle *= M_PI/180; // Convert to radians
    if (angle < 0.001)
    {
      return Eigen::AngleAxisd(0, unitAxis);
    }
    else if (fabs(angle - M_PI) < 0.001)
    {
      return Eigen::AngleAxisd(M_PI, unitAxis);
    }
    else
    {
      return Eigen::AngleAxisd(angle, rotationAxis);
    }
}

void RotationInvariantObjectsRotator::normalizeRotationInvarienceObjects(ObjectSetPtr objectSet, Eigen::Quaterniond quat, const std::vector<std::string>& rotationInvariantTypes)
{
    //    std::cout << "Set normalized to (" << quat.w << ", " << quat.x << ", " << quat.y << ", " << quat.z << ")" << std::endl;
        Eigen::Quaterniond orientationRef = quat;
        Eigen::Vector3d yUnitVector = yUnitVector = Eigen::Vector3d::UnitY();
        orientationRef.normalize();

        for (ISM::ObjectPtr object : objectSet->objects)
        {
           if (std::find(rotationInvariantTypes.begin(), rotationInvariantTypes.end(), object->type) != rotationInvariantTypes.end())
           {
                Eigen::Quaterniond orientation = ISM::GeometryHelper::quatToEigenQuat(object->pose->quat);
                orientation.normalize();
                Eigen::Vector3d objYAxis = ISM::GeometryHelper::getAxisFromQuat(ISM::GeometryHelper::eigenQuatToQuat(orientation), Eigen::Vector3d::UnitY());
                Eigen::Vector3d refYAxis = ISM::GeometryHelper::getAxisFromQuat(ISM::GeometryHelper::eigenQuatToQuat(orientationRef), yUnitVector);
                Eigen::AngleAxisd yTransform = calculateTransform(objYAxis, refYAxis, Eigen::Vector3d::UnitX());
                Eigen::Quaterniond yAlignedOrientation = (yTransform * orientation).normalized();
                Eigen::Vector3d objXAxis = ISM::GeometryHelper::getAxisFromQuat(ISM::GeometryHelper::eigenQuatToQuat(yAlignedOrientation), Eigen::Vector3d::UnitX());
                Eigen::Vector3d refXAxis = ISM::GeometryHelper::getAxisFromQuat(ISM::GeometryHelper::eigenQuatToQuat(orientationRef), Eigen::Vector3d::UnitX());
                object->pose->quat->eigen = (yTransform.inverse() * calculateTransform(objXAxis, refXAxis, Eigen::Vector3d::UnitY())).normalized() * yAlignedOrientation;
            }
        }
}

ObjectPtr RotationInvariantObjectsRotator::getReferenceObject(ObjectSetPtr oset, const std::string& type, const std::string& id)
{
    std::vector<ObjectPtr> objects = oset->objects;
    ObjectPtr currentObject;
    for (unsigned int i = 0; i < objects.size(); i++)
    {
        currentObject = objects.at(i);
        if ((currentObject->type == type) && (currentObject->observedId == id))
        {
            std::cout << "Found reference object." << std::endl;
            return currentObject;
        }
    }
    std::cerr << "Reference object is missing in current Object set" << std::endl;
    return nullptr;
}

}
