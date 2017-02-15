/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "randomDemoRecorder.hpp"

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
        int objectCount;
        bool useClustering;

        double bin_size;
        double maxAngleDeviation;
        int useAPORater;

        TestSpec(int setCount, int objectCount, bool useClustering, double bin_size, double maxAngleDeviation, int useAPORater)
            : setCount(setCount), objectCount(objectCount), useClustering(useClustering), bin_size(bin_size),
              maxAngleDeviation(maxAngleDeviation), useAPORater(useAPORater) {}
};

struct TestResult {
        TestSpec spec;
        double avgRuntime;
        double avgConfidence;
        bool timeout;
};

void removeFileIfExists(const string& filename) {
    if (fs::exists(filename)) {
        fs::remove(filename);
    }
}

void printTest(const TestSpec& t) {
    cerr << "setCount: " << t.setCount << ", objectCount: " << t.objectCount << ", useClustering:" << t.useClustering
         << ",\nbin_size:" << t.bin_size << ", maxAngleDeviation:" << t.maxAngleDeviation
         << ",\nuseAPORater:" << t.useAPORater
         << endl;
}

void printTestResult(const TestResult& r) {
    cerr << "\nTestSpecs: \n\tsetCount: " << r.spec.setCount << ", objectCount: " << r.spec.objectCount << ", useClustering: " << r.spec.useClustering
         << ",\n\tbin_size: " << r.spec.bin_size << ", maxAngleDeviation: " << r.spec.maxAngleDeviation
         << ",\n\tuseAPORater: " << r.spec.useAPORater
         << "\n\nResult: avgRuntime: " << r.avgRuntime << ", avgConfidence: " << r.avgConfidence << ", timeout: " << r.timeout << endl;
}

void writeTestResult(const TestResult& r, ofstream& csvFile) {
    csvFile << r.spec.setCount << "," << r.spec.objectCount << "," << r.spec.useClustering << ","
         << r.spec.bin_size << "," << r.spec.maxAngleDeviation << ","
         << r.spec.useAPORater << ","  << r.avgRuntime << "," << r.avgConfidence << "," << r.timeout << endl;
}

std::string patternName;
int insertedSets;

void trainNewModel(const std::string& sourceFile, const string& testFile, const int& objectCount, const int& setCount, const bool& useClustering)
{
    removeFileIfExists(testFile);
    TableHelper source = TableHelper(sourceFile);
    TableHelper* target = new TableHelper(testFile);
    target->dropTables();
    target->createTablesIfNecessary();

    set<pair<string, string> > typesAndIds;
    patternName = source.getRecordedPatternNames()[0];

    RecordedPatternPtr pattern = source.getRecordedPattern(patternName);
    for (ObjectSetPtr& set : pattern->objectSets)
    {
        for (ObjectPtr& obj : set->objects)
        {
            std::pair<std::string, std::string> p = make_pair(obj->type, obj->observedId);
            if (typesAndIds.find(p) == typesAndIds.end())
            {
                typesAndIds.insert(p);
            }
        }
    }

    set<pair<string, string> > typesAndIdsToUse;
    while (typesAndIdsToUse.size() < (size_t)objectCount)
    {
        set<pair<string, string> >::iterator first = typesAndIds.begin();
        typesAndIdsToUse.insert(*first);
        typesAndIds.erase(first);
    }

    cerr << "creating test database";
    insertedSets = 0;
    for (ObjectSetPtr& set : pattern->objectSets)
    {
        ObjectSetPtr newSet(new ObjectSet());
        for (ObjectPtr& obj : set->objects)
        {
            std::pair<std::string, std::string> p = make_pair(obj->type, obj->observedId);
            if (typesAndIdsToUse.find(p) != typesAndIdsToUse.end())
            {
                newSet->insert(obj);
            }
        }
        target->insertRecordedObjectSet(newSet, patternName);
        insertedSets++;
        cerr << ".";
        cerr.flush();
        if (insertedSets >= setCount) {
            break;
        }
    }
    cerr << " done" << endl;

    delete target;

    Trainer* trainer = new Trainer(testFile);
    if (!useClustering) {
        trainer->setUseClustering(false);
    }
    trainer->trainPattern();
    delete trainer;

    return;
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

    vector<int> setCounts = { 1, 10, 50, 100, 500};
    vector<int> objectCounts = { 1, 2, 3, 4, 5, 10 };
    vector<bool> useClusterings = { true, false };

    vector<double> bin_sizes = { 0.01, 0.1, 0.5 };
    vector<double> maxAngleDeviations = {1.0, 10.0, 45.0};

    //all new options
    vector<int> useAPORaterVec = { 0, 1 };

    queue<TestSpec> tests;
    stack<TestResult> testResults;

    for (int setCount : setCounts)
      {        
        for (int objectCount : objectCounts)
	  {
            for (bool useClustering : useClusterings)
	      {
                for (double bin_size : bin_sizes)
		  {
                    for (double maxAngleDeviation : maxAngleDeviations)
		      {
			for (int useAPORater : useAPORaterVec)
			  {
			    TestSpec t = TestSpec(setCount, objectCount, useClustering, bin_size, maxAngleDeviation, useAPORater);
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
    csvFile << "setCount, objectCount,useClustering,bin_size,maxAngleDeviation,"
            <<"useAPORater,avgRuntime,avgConfidence,timeout"
            << endl;

    //Generate random record db for further use
    RandomDemoRecorder demoRec = *(new RandomDemoRecorder());
    TableHelper* recordingsTable = new TableHelper(sourceFile);
          recordingsTable->dropTables();
          cout<<"generate random record db"<<endl;
          demoRec.generateDemoRecording(sourceFile, objectCounts.back(), setCounts.back(), true);
          cout<<"finished generating"<<endl;
          delete (recordingsTable);

    TestSpec tempTest = tests.front();
    trainNewModel(sourceFile, testFile, tempTest.objectCount, tempTest.setCount, tempTest.useClustering);
    bool usedClusteringBefore = tempTest.useClustering;

    while (!tests.empty()) {        
        TestSpec test = tests.front();
        tests.pop();

        //printTest(test);
        if(usedClusteringBefore != test.useClustering)
        {
            trainNewModel(sourceFile, testFile, test.objectCount, test.setCount, test.useClustering);
            usedClusteringBefore = test.useClustering;
        }


        TestResult testResult = { test, 0, 0, false };
        TableHelper* target = new TableHelper(testFile);
        cerr <<"recognizing";
        int i;
        for (i = 0; ((i < insertedSets) && (i < 25)); i++) {
            ObjectSetPtr testSet = target->getRecordedObjectSet(i+1);
            Recognizer recognizer(testFile, test.bin_size, test.maxAngleDeviation,
                                  test.useAPORater);

            struct timeval start, end;

            gettimeofday(&start, NULL);

            vector<RecognitionResultPtr> results = recognizer.recognizePattern(testSet, 0, 1);

            gettimeofday(&end, NULL);

            long mtime, seconds, useconds;
            seconds = end.tv_sec - start.tv_sec;
            useconds = end.tv_usec - start.tv_usec;

            mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;

            testResult.avgRuntime += mtime;

            for (RecognitionResultPtr& result : results)
            {
                if (result->patternName == patternName)
                {
                    testResult.avgConfidence += result->confidence;
                }
            }
            cerr<<".";
            cerr.flush();

            //if recognition-time longer than 10 sec -> stop further iterations
            if(mtime > 10000)
            {
                testResult.timeout = true;
                break;
            }
        }

        cerr<<"done"<<endl;
        delete (target);

        testResult.avgConfidence /= (double) i;
        testResult.avgRuntime /= (double) i;

        if(testResult.timeout)
        {
            testResult.avgConfidence = 0.0;
            testResult.avgRuntime = 0.0;
        }

        testResults.push(testResult);
        printTestResult(testResult);
        writeTestResult(testResult, csvFile);
    }

    csvFile.close();

    return 0;
}
