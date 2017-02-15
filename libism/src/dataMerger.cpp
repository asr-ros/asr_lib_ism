/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ISM/tools/DataMerger.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <set>

#include <boost/program_options.hpp>

using namespace ISM;
using namespace std;
namespace po = boost::program_options;

int main (int argc, char** argv)
{
    bool mergeRecordings = false;
    bool mergeModels = false;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("target-file,t", po::value<string>()->default_value("record.sqlite"), "target file")
    ;

    po::options_description required("Required");
    required.add_options()
        ("source-file,s", po::value<vector<string> >(), "source file (precede each path by -s)")
    ;

    po::options_description atLeastOne("At least one of these");
    atLeastOne.add_options()
        ("merge-recording,r", po::bool_switch(&mergeRecordings), "merge recordings")
        ("merge-models,m", po::bool_switch(&mergeModels), "merge models")
    ;

    desc.add(required);
    desc.add(atLeastOne);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") || !vm.count("target-file") || !vm.count("source-file") || (!mergeRecordings && !mergeModels))
    {
        cout << desc << "\n";
        return 1;
    }

    bool first = true;
    vector<string> sourceFiles = vm["source-file"].as<vector<string> >();
    string targetFile = vm["target-file"].as<string>();

    cout<<"merging ";
    for (string& source : sourceFiles)
    {
        if (!first)
        {
            cout<<",";
        }
        else
        {
            first = false;
        }
        cout<<source;
    }
    cout<<" into "<<targetFile<<endl;
    cout<<"Type yes to continue"<<endl;
    string yes;
    cin>>yes;
    transform(yes.begin(), yes.end(), yes.begin(), ::tolower);
    if (yes != "yes")
    {
        cout<<"aborting"<<endl;
        return 1;
    }

    //actual merging process
    DataMerger data_merger;
    data_merger.merge(targetFile, sourceFiles,mergeRecordings, mergeModels);
}
