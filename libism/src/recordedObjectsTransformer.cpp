/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ISM/tools/RecordedObjectsTransformer.hpp>

#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <fstream>

/*
 * Command line tool to transform the absolute pose of an entire scene relative to a world coordinate frame.
 * (All absolute poses of recorded objects in a given database are transformed into poses relative to a given object, before the expected pose of this object in world frame is applied on all of them.)
 *
 * Usage:
 * ./recordedObjectsTransformer
 *  -s <path-to-source-database>
 *  -t <path-to-target-database> (does not need to be existence)
 *  -S <reference-object-type>
 *  -I <corresponding-object-id>
 *  -x -y -z <absolut-position-of-reference-object>
 *  -W -X -Y -Z <absolute-orientation-quaternion-of-reference-object>
 *
 * Example:
 * ./recordedObjectsTransformer -s /PATH/SOURCE_DATABASE.sqlite -t /PATH/TARGET_DATABASE.sqlite -S marker_5 -I 052052051100 -x -1.5 -y 0.5 -z 0.7 -W 0.289 -X -0.242 -Y -0.642 -Z 0.667
 *
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
            ("target-file,t", po::value<string>(), "target file")
            ("source-object-type,S", po::value<string>(), "source object type")
            ("source-object-id,I", po::value<string>(), "source object ID")
            ("px,x", po::value<double>(), "Point x")
            ("py,y", po::value<double>(), "Point y")
            ("pz,z", po::value<double>(), "Point z")
            ("qw,W", po::value<double>(), "Quaternion W")
            ("qx,X", po::value<double>(), "Quaternion X")
            ("qy,Y", po::value<double>(), "Quaternion Y")
            ("qz,Z", po::value<double>(), "Quaternion Z")
            ;

    desc.add(required);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") || !vm.count("source-file") || !vm.count("target-file") || !vm.count("source-object-type") || !vm.count("source-object-id")
            || !vm.count("px") || !vm.count("py") || !vm.count("pz") || !vm.count("qw") || !vm.count("qx") || !vm.count("qy") || !vm.count("qz")) {
        cout << desc << "\n";
        return 1;
    }

    string sourceFile = vm["source-file"].as<string>();
    string targetFile = vm["target-file"].as<string>();
    string type = vm["source-object-type"].as<string>();
    string id = vm["source-object-id"].as<string>();
    double px = vm["px"].as<double>();
    double py = vm["py"].as<double>();
    double pz = vm["pz"].as<double>();
    double qw = vm["qw"].as<double>();
    double qx = vm["qx"].as<double>();
    double qy = vm["qy"].as<double>();
    double qz = vm["qz"].as<double>();

    //actual transform process
    RecordedObjectsTransformer objects_transformer;
    objects_transformer.transformRecordedObjects(sourceFile, targetFile, type, id, px, py, pz, qw, qx, qy, qz);
}
