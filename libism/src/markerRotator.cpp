/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ISM/tools/MarkerRotator.hpp>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <fstream>

/*
 * Command line tool to rotate coordinates system of marker_based objects in a given database to be adapted to the current used coordinates system.
 * (CS will be rotated 90 degrees arround the x-axis)
 *
 * Usage:
 * ./markerRotator -s <path-to-source-file> -t <path-to-target-file>
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
    ;

    desc.add(required);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") || !vm.count("source-file") || !vm.count("target-file")) {
        cout << desc << "\n";
        return 1;
    }

    string sourceFile = vm["source-file"].as<string>();
    string targetFile = vm["target-file"].as<string>();


    //actual rotation process
    MarkerRotator marker_rotator;
    marker_rotator.rotateMarker(sourceFile, targetFile);
}
