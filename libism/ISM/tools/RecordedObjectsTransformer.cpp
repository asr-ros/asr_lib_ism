/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "RecordedObjectsTransformer.hpp"
#include "../utility/TableHelper.hpp"
#include "../utility/GeometryHelper.hpp"

#include <fstream>
#include <set>

namespace ISM
{
void RecordedObjectsTransformer::transformRecordedObjects(const std::string& sourceFile, const std::string& targetFile, const std::string& type, const std::string& id,
                                                          double px, double py, double pz, double qw, double qx, double qy, double qz)
{
    Eigen::Vector3d nullPoint = Eigen::Vector3d(0,0,0);
    // Eigen::Quaterniond nullQuat = Eigen::Quaterniond(1,0,0,0);

    Eigen::Vector3d newPoint = Eigen::Vector3d(px,py,pz);
    Eigen::Quaterniond newQuat = Eigen::Quaterniond(qw,qx,qy,qz);


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
    ObjectPtr reference = getReferenceObject(currentSet, type, id);
    if (!reference)
    {
        std::cout << "Reference Object not found!" << std::endl;
        exit(0);
    }
    Eigen::Vector3d transformPoint = nullPoint - reference->pose->point->eigen + newPoint;
    Eigen::Quaterniond transformQuat = newQuat * reference->pose->quat->eigen.inverse();
    Eigen::Vector3d referencePosition = reference->pose->point->eigen;

    double angle = GeometryHelper::getAngleBetweenQuats(GeometryHelper::eigenQuatToQuat(newQuat), reference->pose->quat);
    std::cout << "transformQuat: w: " << transformQuat.w() << ", x: " << transformQuat.x() << ", y: " << transformQuat.y() << ", z: " << transformQuat.z() << std::endl;
    std::cout << "angle: " << angle << std::endl;
    angle *= M_PI/180; // Convert to radians
    while (currentSet->objects.size() != 0)
    {
        std::cout << "Processing object set " << counter << "..." << std::endl;
        transformSet(currentSet, transformPoint, transformQuat, referencePosition);
        targetHelper.insertRecordedObjectSet(currentSet, pattern);
        ++counter;
        currentSet = sourceHelper.getRecordedObjectSet(counter);
    }
}

void RecordedObjectsTransformer::transformSet(ISM::ObjectSetPtr objectSet, Eigen::Vector3d point, Eigen::Quaterniond quat, Eigen::Vector3d orginalCenter)
{
    Eigen::Translation3d translation(orginalCenter);
    Eigen::Translation3d translationBack(-orginalCenter);
    Eigen::Affine3d endTransform;
    endTransform = translation * quat * translationBack;

    for (ISM::ObjectPtr object : objectSet->objects)
    {
        Eigen::Vector3d transformedPoint = object->pose->point->eigen;
        transformedPoint = endTransform * transformedPoint;
        object->pose->point->eigen = transformedPoint + point;

        Eigen::Quaterniond transformedQuat = quat * object->pose->quat->eigen;
        transformedQuat.normalize();
        object->pose->quat->eigen = transformedQuat;
    }
}

void RecordedObjectsTransformer::transformSet(ISM::ObjectSetPtr objectSet, Eigen::Vector3d point, double angle)
{
    Eigen::AngleAxisd axis(angle, Eigen::Vector3d::UnitY());
    for (ISM::ObjectPtr object : objectSet->objects)
    {
        Eigen::Vector3d transformedPoint = object->pose->point->eigen + point;
        Eigen::Quaterniond transformedQuat =  axis * object->pose->quat->eigen;
        transformedQuat.normalize();
        object->pose->point->eigen = transformedPoint;
        object->pose->quat->eigen = transformedQuat;
    }
}

ObjectPtr RecordedObjectsTransformer::getReferenceObject(ObjectSetPtr oset, const std::string& type, const std::string& id)
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
