/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ISM/utility/TableHelper.hpp>
#include <ISM/common_type/RecordedPattern.hpp>
#include <ISM/recognizer/Recognizer.hpp>
#include <boost/program_options.hpp>
#include <string>
#include <set>
#include <stack>
#include <utility>

using namespace ISM;
using namespace std;
namespace po = boost::program_options;

bool detectGeneric = false;
int onlyN = -1;
int maxRuns = -1;
int runs = 0;

void validatePattern(RecordedPatternPtr pattern, RecognizerPtr recognizer) {
    if (maxRuns >= 0 && runs >= maxRuns) {
        return;
    }
    cout << "validate pattern " << pattern->name << endl;
    int idx = 0;
    int setCount = 0;
    int objectCount = 0;
    int identifySum = 0;
    double confidenceSum = 0;
    double thresholdConfidence = 0.8;
    for (ObjectSetPtr& os : pattern->objectSets) {
        if (onlyN >= 0 && onlyN != idx) {
            idx++;
            continue;
        }

        if (maxRuns >= 0 && runs >= maxRuns) {
            break;
        }

        set<pair<string, string> > mappedTypes;

        if (detectGeneric)
        {
            for (ObjectPtr& obj : os->objects) {
                mappedTypes.insert(make_pair(obj->type, obj->observedId));
                objectCount++;
                obj->type = "";
                obj->observedId = "";
            }
        }
        setCount++;
        runs++;

        vector<RecognitionResultPtr> results = recognizer->recognizePattern(os);
        bool found = false;
        for (RecognitionResultPtr& result : results) {
      if (result->patternName == pattern->name) {
                vector<RecognitionResultPtr> all;
                all.push_back(result);
                stack<RecognitionResultPtr> subPatterns;

                for (RecognitionResultPtr& subPattern : result->subPatterns) {
                    subPatterns.push(subPattern);
                }

                while (subPatterns.size() > 0) {
                    RecognitionResultPtr sp = subPatterns.top();
                    subPatterns.pop();
                    all.push_back(sp);
                    for (RecognitionResultPtr& subPattern : sp->subPatterns) {
                        subPatterns.push(subPattern);
                    }
                }

                for (RecognitionResultPtr& pattern : all) {
          for (ObjectPtr& obj : pattern->recognizedSet->objects) {
            set<pair<string, string> >::iterator match = mappedTypes.find(make_pair(obj->type, obj->observedId));
		    if (match != mappedTypes.end()) {
		      mappedTypes.erase(match);
		      identifySum++;
		    }
		  }
                }

                found = true;
                confidenceSum += result->confidence;
                break;
            }
        }
        cout << (found ? "." : ",");
        cout.flush();
        idx++;
        if ((onlyN >= 0 && onlyN - 1 == idx)) {
            break;
        }
    }
    cout << endl;

    double meanConfidence = confidenceSum / (double) setCount;
    cout << (meanConfidence >= thresholdConfidence ? "SUCCESS" : "FAILURE") << " mean confidence for pattern '"
            << pattern->name << "' is " << meanConfidence << endl;
    if (detectGeneric) {
        double identifyRate = (double) identifySum / (double) objectCount;
        cout << "Identification rate: " << identifyRate << endl;
    }
}

int main(int argc, char** argv) {
    double bin_size = 0.005;

    po::options_description desc("Allowed options");
    desc.add_options()("help,h", "produce help message")("database-file,d",
            po::value<string>()->default_value("record.sqlite"), "database file to use")("generic-mode,g",
            po::bool_switch(&detectGeneric),
            "test object type inference by removing object type and id from recognition input")("bin_size,s",
            po::value<double>(&bin_size)->default_value(0.0001), "recognizer bin_size")("pattern-name,p",
            po::value<vector<string> >(), "patters to validate instead of all")("onlyN,o",
            po::value<int>(&onlyN)->default_value(-1), "only the n'th set")("maxRuns,m",
            po::value<int>(&maxRuns)->default_value(-1), "test a maximum of m sets");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    TableHelper t(vm["database-file"].as<string>());
    RecognizerPtr r(new Recognizer(vm["database-file"].as<string>(), bin_size, 10));
    if (vm.count("pattern-name")) {
        vector<string> patternNames = vm["pattern-name"].as<vector<string> >();
        for (string& name : patternNames) {
            RecordedPatternPtr pattern = t.getRecordedPattern(name);
            if (!pattern) {
                cout << "Pattern " << name << " not found!";
                continue;
            }
            validatePattern(pattern, r);
        }
    } else {
        vector<string> patternNames = t.getRecordedPatternNames();
        for (string& name : patternNames) {
            RecordedPatternPtr pattern = t.getRecordedPattern(name);
            if (!pattern) {
                cout << "Pattern " << name << " not found!";
                continue;
            }
            validatePattern(pattern, r);
        }
    }
}
