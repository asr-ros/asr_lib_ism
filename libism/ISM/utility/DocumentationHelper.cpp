/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "DocumentationHelper.hpp"
#include "TableHelper.hpp"
#include "../typedef.hpp"

namespace ISM {

	void DocumentationHelper::writeResult()
	{
		mSVGHelper->writeResult();
	}

	void DocumentationHelper::storeOptimizationRun(std::vector<std::vector<std::pair<TopologyPtr, unsigned int>>>& history,
				CostFunctionPtr<TopologyPtr> globalCostFunction,
				double elapsedRuntime, const std::string& patternName)
	{
		unsigned int numEvaluatedTopologies = 0;
		std::vector<TopologyPtr> selectedTopologies;

		for (unsigned int i = 0; i < history.size(); ++i) {
			std::vector<std::pair<TopologyPtr, unsigned int>> round = history[i];

			numEvaluatedTopologies += round.size();
			for (unsigned int j = 0; j < round.size(); ++j) {
				if (round[j].second != 0)
				{
					selectedTopologies.push_back(round[j].first);
					break;
				}
			}

		}

		mSVGHelper->processHistory(history, globalCostFunction, patternName);

		storeTopologiesToCSV(selectedTopologies, patternName);
		storeMetadataToCSV(numEvaluatedTopologies, history.size(), elapsedRuntime, patternName);

		mSelectedTopologyCounter = 1;
		mReferenceTopology.reset();
	}

	void DocumentationHelper::storeTopologiesToCSV(std::vector<TopologyPtr> selectedTopologies,
			const std::string& patternName)
	{
		std::ostringstream s;
		for (unsigned int i = 0; i < selectedTopologies.size(); ++i) {
			s << i << ", " << selectedTopologies[i]->evaluationResult.falsePositives
				<< ", " << selectedTopologies[i]->evaluationResult.averageRecognitionRuntime << std::endl;
		}
		path filePath = mOutputPath / CSV_SUBFOLDER_NAME / (patternName + "_plot.csv");

		writeToFile(filePath, s.str());
	}

	void DocumentationHelper::storeMetadataToCSV(unsigned int numEvaluatedTopologies, unsigned int numOptimizationRounds,
			double elapsedRuntime, const std::string& patternName)
	{
		std::ostringstream s;
		s << numEvaluatedTopologies << " , " << numOptimizationRounds << " , " << elapsedRuntime;
		path filePath = mOutputPath / CSV_SUBFOLDER_NAME / (patternName + "_metadata.csv");

		writeToFile(filePath, s.str());
	}

	void DocumentationHelper::storeTopology(TopologyPtr toStore, const std::string& patternName,
			const std::string& topologyName, std::map<std::string, std::vector<ISM::VoteSpecifierPtr>> objectDefinitons)
	{
		path filePath = mOutputPath / TOPOLOGIES_SUBFOLDER_NAME / patternName;
		if(!boost::filesystem::exists(filePath))
			boost::filesystem::create_directories(filePath);

		filePath /= topologyName;
		if(!boost::filesystem::exists(filePath))
			boost::filesystem::create_directories(filePath);

		ObjectRelations objectRelations = toStore->objectRelations;
		std::ostringstream os;

		os << patternName;
		for (ObjectRelations::iterator it = objectRelations.begin(); it != objectRelations.end(); ++it)
		{
			os << ":" << std::to_string(it->first);
		}
		writeToFile(filePath / "startFile.txt", os.str());

		os.str("");

		EvaluationResult er = toStore->evaluationResult;
		os << "False Positives, False Negatives, Average Evaluation Duration" << std::endl;
		os << er.falsePositives << ", " << er.falseNegatives << ", " << er.averageRecognitionRuntime;
		writeToFile(filePath / "evaluationResult.csv", os.str());

		mDotHelper->storeISMToDot(filePath, patternName, topologyName, objectDefinitons);

		if (mReferenceTopology)
		{
			mDotHelper->storeRelationsToDot(filePath, patternName, topologyName,
					objectRelations, mReferenceTopology->objectRelations);
		}
		else
		{
			mDotHelper->storeRelationsToDot(filePath, patternName, topologyName, objectRelations);
		}
	}
	

	void DocumentationHelper::writeToFile(path filePath, const std::string& content)
	{
		std::ofstream file;
		std::ios_base::iostate exceptionMask = file.exceptions() | std::ios::failbit | std::ios::badbit;
		file.exceptions(exceptionMask);
		try
		{
			file.open(filePath.string());
			file << content;
			file.flush();
			file.close();
		}
		catch (std::ios_base::failure& e)
		{
			std::cerr << e.what() << "\n";
		}
	}

    void DocumentationHelper::writeIsmToDB(path dbPath, const IsmPtr & ism)
	{
		if (!mStoreISM) return;
		LogHelper::logMessage("\nStoring ISM to : " + dbPath.string());

		if(!boost::filesystem::exists(dbPath.parent_path()))
			boost::filesystem::create_directories(dbPath.parent_path());

		if (!boost::filesystem::exists(dbPath))
		{
			boost::filesystem::copy(mSourceDBPath, dbPath);
		}

		try
		{
			TableHelperPtr localTableHelper(new TableHelper(dbPath.string()));
			localTableHelper->createTablesIfNecessary();

			LogHelper::displayProgress(0);

            for (const std::string& typeIt : ism->objectTypes)
			{
				localTableHelper->ensureModelObjectType(typeIt);
			}
            for (const std::pair<std::string, PatternPtr>& patternIt : ism->patternDefinitions)
			{
				localTableHelper->upsertModelPattern(patternIt.second->name,
						patternIt.second->expectedMaxWeight);
			}

			unsigned int numVotes = 0;
			unsigned int numVotesInserted = 0;

            for (const std::pair<std::string, std::vector<VoteSpecifierPtr> >& objectTypeIt : ism->voteSpecifiersPerObject)
			{
				numVotes += objectTypeIt.second.size();
			}

            for (std::pair<const std::string, std::vector<VoteSpecifierPtr> >& objectTypeIt : ism->voteSpecifiersPerObject)
			{
				if (objectTypeIt.second.size() == 0)
				{
					continue;
				}
                for (VoteSpecifierPtr& voteIt : objectTypeIt.second)
				{
					localTableHelper->insertModelVoteSpecifier(voteIt);
					numVotesInserted++;
					LogHelper::displayProgress(((double) numVotesInserted) / numVotes);
				}

			}
		}
		catch (soci::soci_error& e)
		{
			std::cerr<<"soci error\n"<<e.what()<<std::endl;
		}
	}

	void DocumentationHelper::storeIsm(const IsmPtr& ism)
	{
        writeIsmToDB(mSourceDBPath, ism);
	}

	void DocumentationHelper::storeIsm(const std::string& filename, const IsmPtr& ism)
	{
		if (!mStoreISM) return;

		path dbPath = mOutputPath / "SQL" / (filename + ".sqlite");
        writeIsmToDB(dbPath, ism);
	}

	void DocumentationHelper::setReferenceTopology(TopologyPtr topology, const std::string & patternName)
	{
		std::string topologyName = "selected_topology_" + std::to_string(mSelectedTopologyCounter);
		path filePath = mOutputPath / TOPOLOGIES_SUBFOLDER_NAME / patternName / SELECTED_TOPOLOGIES_FOLDER;

		if(!boost::filesystem::exists(filePath))
			boost::filesystem::create_directories(filePath);

		if (mReferenceTopology)
		{
			mDotHelper->storeRelationsToDot(filePath, patternName, topologyName,
					topology->objectRelations, mReferenceTopology->objectRelations);
		}
		else
		{
			mDotHelper->storeRelationsToDot(filePath, patternName, topologyName, topology->objectRelations);
		}

		mReferenceTopology = topology;
		mSelectedTopologyCounter++;
	}
}
