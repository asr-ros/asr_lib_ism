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
#include <ISM/heuristic_trainer/Trainer.hpp>
#include <boost/program_options.hpp>
#include <string>
#include <set>
#include <stack>
#include <queue>
#include <utility>
#include <boost/filesystem.hpp>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <iostream>

using namespace ISM;
using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct TestSpec {
        int setCount;
        bool useClustering;
        int objectCount;
        double bin_size;
        bool useId;
        bool useType;
};

struct TestResult {
        TestSpec spec;
        double avgRuntime;
        double avgConfidence;
};

void removeFileIfExists(const string& filename) {
    if (fs::exists(filename)) {
        fs::remove(filename);
    }
}

void printTest(const TestSpec& t) {
    cerr << t.setCount << "," << t.objectCount << "," << t.bin_size << "," << t.useClustering
            << "," << t.useType << "," << t.useId << endl;
}

void printTestResult(const TestResult& r) {
    cerr << r.spec.setCount << "," << r.spec.objectCount << "," << r.spec.bin_size << "," << r.spec.useClustering
            << "," << r.spec.useType << "," << r.spec.useId << "," << r.avgRuntime << "," << r.avgConfidence << endl;
}

void writeTestResult(const TestResult& r, ofstream& csvFile) {
        csvFile << r.spec.setCount << "," << r.spec.objectCount << "," << r.spec.bin_size << "," << r.spec.useClustering
                << "," << r.spec.useType << "," << r.spec.useId << "," << r.avgRuntime << "," << r.avgConfidence
                << endl;
}

int main(int argc, char** argv) {
    po::options_description desc("Allowed options");
    desc.add_options()("help,h", "produce help message")
      ("database-file,d", po::value<string>()->default_value("record.sqlite"), "database file to use")
      ("test-database-file,t", po::value<string>()->default_value("test.sqlite"), "database file write tests to")
      ("csv-file,c", po::value<string>()->default_value("results.csv"), "csv file to write results");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cerr << desc << "\n";
        return 0;
    }

    vector<int> setCounts = { 10, 50, 100, 150, 200, 250, 300, 350, 400 };
    vector<bool> useClusterings = { true, false };
    vector<int> objectCounts = { 1, 2, 3, 4, 5, 6 };
    vector<double> bin_sizes = {
            0.5, 0.4, 0.3, 0.2, 0.1,
            0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01
    };
    vector<bool> useIds = { true };
    vector<bool> useTypes = { true };

    queue<TestSpec> tests;
    stack<TestResult> testResults;

    for (const int& setCount : setCounts) {
        for (const bool& useClustering : useClusterings) {
            for (const int& objectCount : objectCounts) {
                for (const double& bin_size : bin_sizes) {
                    for (const bool& useId : useIds) {
                        for (const bool& useType : useTypes) {
                            TestSpec t = { setCount, useClustering, objectCount, bin_size, useId, useType };
                            tests.push(t);
                        }
                    }
                }
            }
        }
    }


    string sourceFile = vm["database-file"].as<string>();
    string testFile = vm["test-database-file"].as<string>();

    string csvFilename = vm["csv-file"].as<string>();
    removeFileIfExists(csvFilename);
    ofstream csvFile;
    csvFile.open(csvFilename.c_str());
    csvFile << "setCount,objectCount,bin_size,useClustering,useType,useId,runtime,confidence" << endl;

    while (!tests.empty()) {
        removeFileIfExists(testFile);
        TestSpec test = tests.front();
        tests.pop();

        printTest(test);

        TableHelper source = TableHelper(sourceFile);
        TableHelper* target = new TableHelper(testFile);
        set<pair<string, string> > typesAndIds;
        string patternName = source.getRecordedPatternNames()[0];

        const RecordedPatternPtr pattern = source.getRecordedPattern(patternName);
        for (ObjectSetPtr& set : pattern->objectSets)
        {
            for (ObjectPtr& obj : set->objects)
            {
                pair<string, string> p = make_pair(obj->type, obj->observedId);
                if (typesAndIds.find(p) == typesAndIds.end()) {
                    typesAndIds.insert(p);
                }
            }
        }

        set<pair<string, string> > typesAndIdsToUse;
        while (typesAndIdsToUse.size() < (size_t)test.objectCount) {
            set<pair<string, string> >::iterator first = typesAndIds.begin();
            typesAndIdsToUse.insert(*first);
            typesAndIds.erase(first);
        }

        cerr << "creating test database";
        int insertedSets = 0;
        string patternNameToUse;
        for (ObjectSetPtr& set : pattern->objectSets)
        {
            ObjectSetPtr newSet(new ObjectSet());
            for (ObjectPtr& obj : set->objects)
            {
                pair<string, string>  p = make_pair(obj->type, obj->observedId);
                if (typesAndIdsToUse.find(p) != typesAndIdsToUse.end()) {
                    newSet->insert(obj);
                }
            }
            target->insertRecordedObjectSet(newSet, patternName);
            insertedSets++;
            cerr << ".";
            cerr.flush();
            if (insertedSets >= test.setCount) {
                break;
            }
        }
        cerr << " done" << endl;

        delete (target);

        Trainer* trainer = new Trainer(testFile);
        if (!test.useClustering) {
            trainer->setUseClustering(false);
        }
        trainer->trainPattern();
        delete trainer;

        TestResult testResult = { test, 0, 0 };
        target = new TableHelper(testFile);
        cerr <<"recognizing";
        for (int i = 0; i < insertedSets; i++) {
            const ObjectSetPtr testSet = target->getRecordedObjectSet(i);

            for (ObjectPtr& obj : testSet->objects) {
                if (!test.useType) {
                    obj->type = "";
                }
                if (!test.useId) {
                    obj->observedId = "";
                }
            }

            Recognizer recognizer(testFile, test.bin_size, 10, true);

            struct timeval start, end;

            gettimeofday(&start, NULL);

            vector<RecognitionResultPtr> results = recognizer.recognizePattern(testSet, 0, 1);

            gettimeofday(&end, NULL);

            long mtime, seconds, useconds;
            seconds = end.tv_sec - start.tv_sec;
            useconds = end.tv_usec - start.tv_usec;

            mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;

            testResult.avgRuntime += mtime;

            for (RecognitionResultPtr& result : results) {
          if (result->patternName == patternName) {
        testResult.avgConfidence += result->confidence;
                }
            }
            cerr<<".";
            cerr.flush();
        }

        cerr<<"done"<<endl;
        delete (target);

        testResult.avgConfidence /= (double) insertedSets;
        testResult.avgRuntime /= (double) insertedSets;

        testResults.push(testResult);
        printTestResult(testResult);
        writeTestResult(testResult, csvFile);
    }

    csvFile.close();

    return 0;
}
