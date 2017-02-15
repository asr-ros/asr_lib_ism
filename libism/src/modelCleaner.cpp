/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ISM/tools/DataCleaner.hpp>
#include <string>
#include <algorithm>
#include <boost/program_options.hpp>

using namespace ISM;
using namespace std;
namespace po = boost::program_options;

int main (int argc, char** argv)
{
    po::positional_options_description p;
    p.add("database-file", -1);

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("database-file,d", po::value<vector<string> >(), "database file")
    ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help") || !vm.count("database-file"))
    {
        cout << desc << "\n";
        return 1;
    }

    vector<string> databaseFiles = vm["database-file"].as<vector<string> >();

    cout<<"removing models from ";

    bool first = true;
    for (string& database : databaseFiles)
    {
        if (!first)
        {
            cout<<",";
        }
        else
        {
            first = false;
        }
        cout<<database;
    }
    cout<<" ?"<<endl;
    cout<<"Type yes to continue"<<endl;
    string yes;
    cin>>yes;
    transform(yes.begin(), yes.end(), yes.begin(), ::tolower);
    if (yes != "yes")
    {
        cout<<"aborting"<<endl;
        return 1;
    }

    //actual cleaning process
    DataCleaner data_cleaner;
    data_cleaner.cleanModels(databaseFiles);
}
