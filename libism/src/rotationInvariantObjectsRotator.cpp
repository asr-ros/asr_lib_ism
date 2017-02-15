/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ISM/tools/RotationInvariantObjectsRotator.hpp>

#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <fstream>

/*
 * Command line tool to rotate coordinates system of rotation invariant objects in a given database, so that an object has a fixed
 * orientation to the map frame or to a reference object.
 * - Currently just objects of type Cup and PlateDeep are normalized to avoid dependencies to object_database ROS package.
 * - If rotation to map frame is asked, tool expects data to be already given in map frame.
 *
 * Usage:
 * - map frame as reference:
 *     ./rotationInvariantRotator -s <path-to-source-file> -t <path-to-target-file>
 *
 * - object as reference:
 *     ./rotationInvariantRotator -s <path-to-source-file> -t <path-to-target-file> -i <object-identifier> -o <object-type>
 */

using namespace ISM;
using namespace std;
namespace po = boost::program_options;


int main (int argc, char** argv) {
    po::options_description desc("Allowed options");
    desc.add_options()           
            ("help,h", "produce help message")
            ;

    po::options_description required("Required");
    required.add_options()
        ("source-file,s", po::value<string>(), "source file")
        ("target-file,t", po::value<string>(), "target file");
    desc.add(required);

    po::options_description optional("Optional");
    optional.add_options()
            ("object-type,o", po::value<string>(), "Object type")
            ("object-id,i", po::value<string>(), "Object id");
    desc.add(optional);


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") || !vm.count("source-file") || !vm.count("target-file")) {
        cout << desc << endl;
        return 1;
    }

    string objectType;
    string objectId;

    bool useMapFrame = false;
    if (!vm.count("object-type") || !vm.count("object-id")) {
        cout << "object-type/object-id are not set" << endl;
        cout << "!!!!!!!Reference object is map frame!!!!!!!" << endl;
        useMapFrame = true;
    } else {
        objectType = vm["object-type"].as<string>();
        objectId = vm["object-id"].as<string>();
    }

    string sourceFile = vm["source-file"].as<string>();
    string targetFile = vm["target-file"].as<string>();

    vector<string> rotationInvariantTypes;
    rotationInvariantTypes.push_back("Cup");
    rotationInvariantTypes.push_back("PlateDeep");

    //actual rotation process
    RotationInvariantObjectsRotator rotator;
    rotator.rotateRotationInvariantObjects(sourceFile, targetFile, objectType, objectId, useMapFrame, rotationInvariantTypes);

}
