/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#include <ISM/utility/TableHelper.hpp>
#include <ISM/recognizer/Recognizer.hpp>

#define ISM_DATA "/media/share/data"
using namespace ISM;
using namespace std;

void drawISM(string model, string patternName, map<string, map<string, unsigned> >graphVizData, unsigned testsPerformed, unsigned timeSteps)
{
  if (testsPerformed == 0)
    {
      testsPerformed = 1;
    }
  ofstream file;
  stringstream filePath;
  filePath<<std::string(ISM_DATA)+"/evaluation/gedeck_evaluation/complexity/"<<patternName<<"_"<<model<<"_ISMWithVotes.dot";
  string filename = filePath.str();
  filePath.str("");
  vector<pair<string, string> > alreadyTakenCombinations;
  ios_base::iostate exceptionMask = file.exceptions() | ios::failbit | ios::badbit;
  file.exceptions(exceptionMask);
  //map<string, set<string> > referencesPerObject;
  try
    {
      file.open(filename);
      file<<"digraph "<<model<<" {\n";
      for (const pair<string, map<string, unsigned> >& type : graphVizData)
	{
      for (const pair<string, unsigned>& voteIt : type.second)
	    {
	      file<<voteIt.first<<"[shape=\"box\"];\n";
	      //referencesPerObject[type.first].insert(voteIt.first);
	    }
	}
      for (const pair<string, map<string, unsigned> >& typeIt : graphVizData)
	{
      for (const pair<string, unsigned>& voteIt : typeIt.second)
	    {
	      if (find(alreadyTakenCombinations.begin(), alreadyTakenCombinations.end(), make_pair(voteIt.first, typeIt.first)) == alreadyTakenCombinations.end())
		{
		  //cout<<"from "<<typeIt.first<<" to "<<voteIt.first<<": "<<voteIt.second<<endl;exit(0);
		  file<<voteIt.first<<" -> "<<typeIt.first<<"[ label=\" &#8746;"
		      <<(unsigned)((((double)voteIt.second) / testsPerformed) / timeSteps)<<"&#124;&#8594;"
		    //<<(unsigned)((((double)typeIt.second.size() / testsPerformed) / timeSteps) / referencesPerObject.at(typeIt.first).size())<<"&#124;&#8594;"
		      <<(unsigned)(((double)voteIt.second) / testsPerformed)
		    //<<(unsigned)(((double)typeIt.second.size() / testsPerformed) / referencesPerObject.at(typeIt.first).size())
		      <<"\", decorate=\"true\", labelfoat=\"true\", labelfontcolor=\"red\", dir=\"back\"];\n";
		  alreadyTakenCombinations.push_back(make_pair(voteIt.first, typeIt.first));
		}
	    }	  
	}
      file<<"}\n\n";
      file.flush();
      file.close();
    }
  catch (ios_base::failure& e)
    {
      cerr<<e.what()<<"\n";
      //exit(1);
    }
}

void addVizData(std::map<std::string, std::map<std::string, unsigned> >& graphVizData, const RecognizerPtr& recognizer)
{
  std::map<ObjectPtr, std::vector<VotedPosePtr> > votingCache = recognizer->getVotingCache();
  for (const std::pair<ObjectPtr, std::vector<VotedPosePtr> >& it : votingCache)
    {
      for (const VotedPosePtr& voteIt : it.second)
	{
      ++graphVizData[it.first->type][voteIt->vote->patternName];
	}
    }    
}


int main ()
{
  std::vector<std::pair<std::string, std::string> > modeldbs = 
    {
      std::make_pair("best", std::string(ISM_DATA)+"/evaluation/gedeck_evaluation/sourcedbs/record.sqlite.combinatorialTrainer_example_1_best"),
      std::make_pair("best_naive", std::string(ISM_DATA)+"/evaluation/gedeck_evaluation/sourcedbs/record.sqlite.combinatorialTrainer_example_1_best_naive"),
      std::make_pair("star", std::string(ISM_DATA)+"/evaluation/gedeck_evaluation/sourcedbs/record.sqlite.combinatorialTrainer_example_1_star"),
      std::make_pair("fully_meshed", std::string(ISM_DATA)+"/evaluation/gedeck_evaluation/sourcedbs/record.sqlite.combinatorialTrainer_example_1_fullyMeshed"),
      std::make_pair("fully_meshed_naive",std::string(ISM_DATA)+"/evaluation/gedeck_evaluation/sourcedbs/record.sqlite.combinatorialTrainer_example_1_fullyMeshed_naive")
    };
  ofstream output;
  output.open(std::string(ISM_DATA)+"/evaluation/gedeck_evaluation/output.csv");
  output<<"model, avgTime, falsePositives\n";      
   for (std::pair<std::string, std::string>& model : modeldbs)
    {
      std::map<std::string, std::map<std::string, unsigned> > vizData;
      cout<<"trying to open db file\n";
      TableHelperPtr validSetsTable(new TableHelper(std::string(ISM_DATA)+"/evaluation/gedeck_evaluation/sourcedbs/record.sqlite.combinatorialTrainer_example_1_validTestSets"));
      cout<<"could open valid sets\n";
      const RecordedPatternPtr validPatterns = validSetsTable->getRecordedPattern("combinatorialTrainer_example_1");
      vector<ObjectSetPtr> validSets;
      if (validPatterns != 0)
	{
	  validSets = validPatterns->objectSets;
	}
      struct timeval start, end;
      long time = 0;
      TableHelperPtr invalidSetsTable(new TableHelper(std::string(ISM_DATA)+"/evaluation/gedeck_evaluation/sourcedbs/record.sqlite.combinatorialTrainer_example_1_invalidTestSets"));
      cout<<"could open invalid sets\n ";
      const RecordedPatternPtr invalidPatterns = invalidSetsTable->getRecordedPattern("combinatorialTrainer_example_1");
      vector<ObjectSetPtr> invalidSets;
      if (invalidPatterns != 0)
    {
	  invalidSets = invalidPatterns->objectSets;
	}  
      cout<<invalidSets.size()<<endl;
      RecognizerPtr recog(new Recognizer(model.second, 0.03, 10, false));
      cout<<"could create recognizer\n";
      for (ObjectSetPtr& validSet : validSets)
	{
	  if (model.first == "fully_meshed_naive")
	    {
	      break;
	    }
	  cout<<".";cout.flush();
	  gettimeofday(&start, NULL);
      const std::vector<RecognitionResultPtr> res = recog->recognizePattern(validSet, 0.0, 1);
	  gettimeofday(&end, NULL);
	  time += ((end.tv_sec - start.tv_sec) * 1000) + ((end.tv_usec - start.tv_usec) / 1000);
	  addVizData(vizData, recog);
	}
      unsigned fps = 0;
      for (ObjectSetPtr& invalidSet : invalidSets)
	{
	  cout<<".";cout.flush();
	  gettimeofday(&start, NULL);
      const std::vector<RecognitionResultPtr> res = recog->recognizePattern(invalidSet, 0.0, 1);
	  gettimeofday(&end, NULL);
	  time += ((end.tv_sec - start.tv_sec) * 1000) + ((end.tv_usec - start.tv_usec) / 1000);
	  addVizData(vizData, recog);
      if (res.size() > 0 && res[0]->confidence == 1 && res[0]->patternName == "combinatorialTrainer_example_1")
	    {
	      ++fps;
	    }
	  if (model.first == "fully_meshed_naive")
	    {
	      break;
	    }
	}
      double avg;
      if (model.first == "fully_meshed_naive")
	{
      drawISM(model.first, "combinatorialTrainer_example_1", vizData, 1, 112.0);
	  avg = ((double)time) / 1.0;
	}
      else
	{
      drawISM(model.first, "combinatorialTrainer_example_1", vizData, validSets.size() + invalidSets.size(), 112.0);
	  avg = ((double)time) / (validSets.size() + invalidSets.size());
	}
      cout<<fps<<endl;
      output<<model.first<<","<<avg<<","<<fps<<endl;
    }
  output.close();
}

