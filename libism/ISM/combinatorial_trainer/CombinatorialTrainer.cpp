/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "CombinatorialTrainer.hpp"

#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/posix_time/posix_time_io.hpp"
#include "boost/algorithm/string.hpp"

#include "utility/LogHelper.hpp"
#include "utility/SVGHelper.hpp"
#include "utility/DocumentationHelper.hpp"

#include <iostream>
#include <limits>
#include <ctime>

#include "OptimizationRunnerFactory.hpp"

namespace ISM
{
	using boost::filesystem::path;

	CombinatorialTrainer::CombinatorialTrainer(CombinatorialTrainerParameters params)
	{
		mParams = params;
		mDBPath = params.general.dbfilename;

		if (!boost::filesystem::exists(mDBPath))
		{
			std::stringstream ss;
			ss << mDBPath << " does not exist!";
			throw std::runtime_error(ss.str());
		}

		mOutputDataPath = params.general.outputDataPath;
		if(!boost::filesystem::is_directory(mOutputDataPath))
		{
			std::stringstream ss;
			ss << mOutputDataPath << " does not exist or is not a directory!";
			throw std::runtime_error(ss.str());
		}

		const std::string runName = mDBPath.stem().string() + "_" + genTimeString();
		mOutputDataPath /= runName;


		tableHelper = ISM::TableHelperPtr(new ISM::TableHelper(params.general.dbfilename));
		tableHelper->dropModelTables();

		objectTracksPerPattern = getRecordedObjectsTracks();
		allObjectRelationsPerPattern = calculateAllObjectRelations();
		patternNames = tableHelper->getRecordedPatternNames();
		initStartTopologiesPerPattern(params.general.loadStartTopologiesFrom);

		//Set up LogHelper
		std::string logFileName = "Log_" + runName + ".txt";
		path logFilePath = mOutputDataPath / "Logfile" / logFileName;
		LogHelper::init(logFilePath, LOG_INFO);

		mFullyMeshedTopologyPerPattern = learnFullyMeshedTopologyPerPattern();

		initTestSets(params.evaluator.binSize, params.evaluator.maxAngleDeviation, params.evaluator.confidenceThreshold,
				params.general.loadValidTestSetsFrom, params.general.loadInvalidTestSetsFrom, params.general.testSetCount);

		bool storeValidTestSets = !params.general.storeValidTestSetsTo.empty();
		bool storeInvalidTestSets = !params.general.storeInvalidTestSetsTo.empty();

		if (storeValidTestSets || storeInvalidTestSets)
		{
			LogHelper::logMessage("Storing test sets: \n");
			if (storeValidTestSets)
			{
				path dbFilePath(params.general.storeValidTestSetsTo);
				storeTestSetsToDB(mTestSets.first, dbFilePath, "valid");
			}
			if (storeInvalidTestSets)
			{
				path dbFilePath(params.general.storeInvalidTestSetsTo);
				storeTestSetsToDB(mTestSets.second, dbFilePath, "invalid");
			}
			LogHelper::logLine();
		}
	}

	std::map<std::string, std::pair<double, TreePtr> >CombinatorialTrainer::learn()
	{
		mDocumentationHelper = DocumentationHelperPtr(new DocumentationHelper(mOutputDataPath, mDBPath,
                    objectTracksPerPattern));

		OptimizationRunnerPtr optimizationRunner = OptimizationRunnerFactory::createOptimizationRunner(
					mParams.evaluator, mParams.topologyGenerator,
					mParams.optimizationAlgorithm, mParams.costFunction,
					mParams.treeValidator, mTestSets,
					allObjectRelationsPerPattern, mDocumentationHelper,
					mParams.general.storeFullyMeshedISM, mParams.general.storeStartTopologyISM);

		LogHelper::logMessage("Training has started!");

		std::map<std::string, std::pair<double, TreePtr> > bestPerPattern;
		std::map<std::string, IsmPtr> bestISMPerPattern;

        for (std::string& patternName : patternNames)
		{
			//Learn best Tree
			std::pair<double, TreePtr> currentBest;
			if (mStartTopologiesPerPattern.find(patternName) != mStartTopologiesPerPattern.end())
			{
				double bestEvaluationResult = std::numeric_limits<double>::max();
				std::vector<TopologyPtr> startTopologies = mStartTopologiesPerPattern[patternName];
				for (unsigned int i = 0; i < startTopologies.size(); ++i)
				{
					std::pair<double, TreePtr> result =
						optimizationRunner->runOptimization(patternName, startTopologies[i]);

					if (result.first >= 0 && result.first < bestEvaluationResult)
					{
						currentBest = result;
						bestEvaluationResult = result.first;
					}
				}
			}
			else
			{
					currentBest = optimizationRunner->runOptimization(patternName);
			}

			if (currentBest.second)
			{
				IsmPtr bestISM = currentBest.second->getISM();

				bestPerPattern.insert(std::make_pair(patternName, currentBest));
				bestISMPerPattern.insert(std::make_pair(patternName, bestISM));

				mDocumentationHelper->storeIsm("optimized", bestISM);
				mDocumentationHelper->storeIsm(bestISM);
			}
		}

		mDocumentationHelper->writeResult();

		LogHelper::logMessage("Training is done!");
		LogHelper::close();

		return bestPerPattern;
	}

	const std::map<std::string, ISM::TracksPtr> CombinatorialTrainer::getRecordedObjectsTracks()
	{
		std::vector<std::string> patternNames = tableHelper->getRecordedPatternNames();
		std::vector<ISM::ObjectSetPtr> objectsInCurrentPattern;
		ISM::TracksPtr tracksInCurrentPattern;
		std::map<std::string, ISM::TracksPtr> objectTracksPerPattern;
        for (std::string& patternNameIt : patternNames)
		{
			objectsInCurrentPattern = tableHelper->getRecordedPattern(patternNameIt)->objectSets;
			tracksInCurrentPattern = ISM::TracksPtr(new ISM::Tracks(objectsInCurrentPattern));
			objectTracksPerPattern.insert(std::make_pair(patternNameIt, tracksInCurrentPattern));
			for (size_t it = 0; it < tracksInCurrentPattern->tracks.size(); ++it)
			{
				if (tracksInCurrentPattern->tracks[it]->objects.size() > 0)
				{
					bool foundObjectModel = false;
                    for(ObjectPtr& o : tracksInCurrentPattern->tracks[it]->objects)
					{
						if(o)
						{
							objectModelsPerPattern[patternNameIt].insert(
                                    std::make_pair(o->type, o->ressourcePath));
							foundObjectModel = true;
							break;
						}
					}
					if(!foundObjectModel)
					{
						std::cerr << "CombinatorialTrainer::getRecordedObjectsTracks: "
							<< " Not one Object in track." << std::endl;
					}
				}
				if (it < tracksInCurrentPattern->tracks.size() - 1 &&
						!tracksInCurrentPattern->tracks[it]->objects.size() ==
						tracksInCurrentPattern->tracks[it + 1]->objects.size())
				{
					std::cerr<<"Corrupt database\n";
					exit(-6);
				}
			}
		}

		return objectTracksPerPattern;
	}

	const std::map<std::string, ISM::ObjectRelations> CombinatorialTrainer::calculateAllObjectRelations()
	{
		std::map<std::string, ISM::ObjectRelations> allObjectRelationsPerPattern;
        for (const std::pair<std::string, ISM::TracksPtr>& patternIt : objectTracksPerPattern)
		{
			ISM::ObjectRelations inCurrentPattern;
			std::vector<ISM::ObjectRelationPtr> selfiesInCurrentPattern;
			unsigned relationId = 0;
            for (ISM::TrackPtr& tracksIt : patternIt.second->tracks)
			{
                for (ISM::TrackPtr& otherTracksIt  : patternIt.second->tracks)
				{
					if (otherTracksIt->observedId != tracksIt->observedId || otherTracksIt->type != tracksIt->type)
					{
						bool alreadyThere = false;
                        for (const std::pair<unsigned int, ISM::ObjectRelationPtr>& inCurrentPatternIt : inCurrentPattern)
						{
							if ((inCurrentPatternIt.second->getObjectIdA() == tracksIt->observedId &&
										inCurrentPatternIt.second->getObjectIdB() == otherTracksIt->observedId &&
										inCurrentPatternIt.second->getObjectTypeA() == tracksIt->type &&
										inCurrentPatternIt.second->getObjectTypeB() ==  otherTracksIt->type)
									|| (inCurrentPatternIt.second->getObjectIdA() == otherTracksIt->observedId &&
										inCurrentPatternIt.second->getObjectIdB() ==  tracksIt->observedId &&
										inCurrentPatternIt.second->getObjectTypeA() == otherTracksIt->type &&
										inCurrentPatternIt.second->getObjectTypeB() == tracksIt->type))
							{
								alreadyThere = true;
							}
						}
						if (!alreadyThere)
						{
							//ObjectRelation constructs relations between objects at certain snapshots
							ObjectRelationPtr objectRelation = ObjectRelationPtr(
									new ObjectRelation(tracksIt, otherTracksIt, patternIt.first));
							inCurrentPattern.insert(std::make_pair(relationId++, objectRelation));
						}
					}
					else
					{
						ObjectRelationPtr objectRelation = ISM::ObjectRelationPtr(
								new ISM::ObjectRelation(tracksIt, patternIt.first));
						selfiesInCurrentPattern.push_back(objectRelation);
					}
				}
			}

			allObjectRelationsPerPattern.insert(std::make_pair(patternIt.first, inCurrentPattern));
			this->allSelfRelationsPerPattern.insert(std::make_pair(patternIt.first, selfiesInCurrentPattern));
		}
		//We have to assert that object appearance are clustered by objects. This is needed for the correct order of ISM construction
		assert(checkCorrectOrder(allObjectRelationsPerPattern));
		return allObjectRelationsPerPattern;
	}

	bool CombinatorialTrainer::checkCorrectOrder(std::map<std::string, ISM::ObjectRelations> allObjectRelationsPerPattern)
	{
        for (const std::pair<std::string, ISM::ObjectRelations>& pattern : allObjectRelationsPerPattern)
		{
            for (const std::pair<unsigned int, ISM::ObjectRelationPtr>& relation : pattern.second)
			{
				bool appeared = false;
				bool disappeared = false;
				bool reappeared = false;
                for (const std::pair<unsigned int, ISM::ObjectRelationPtr>& otherRelation : pattern.second)
				{
					if (otherRelation.second->getObjectTypeA() == relation.second->getObjectTypeA() &&
							otherRelation.second->getObjectIdA() == relation.second->getObjectIdA())
					{
						appeared = true;
						if (disappeared)
						{
							reappeared = true;
						}
					}
					else if (appeared)
					{
						disappeared = true;
					}
					if (reappeared)
					{
                        for (const std::pair<unsigned int, ISM::ObjectRelationPtr>& it : pattern.second)
						{
							std::cout<<it.second<<std::endl;
						}
					}
					if (reappeared)
					{
						LogHelper::logMessage("Wrong order of relations. We have to ASSERT the correct order\n",
								LOG_ERROR, LogHelper::LOG_COLOR_RED);
						return false;
					}
				}
			}
		}
		return true;
	}

	bool CombinatorialTrainer::containsAllObjects(const ISM::ObjectRelations& topology,
			const std::string& patternName) const
	{
		const ISM::TracksPtr allObjects = objectTracksPerPattern.at(patternName);
        for (ISM::TrackPtr& object : allObjects->tracks)
		{
			bool hasAppearance = false;
            for (const std::pair<unsigned int, ISM::ObjectRelationPtr>& relation : topology)
			{
				if (relation.second->containsObject(object->type, object->observedId))
				{
					hasAppearance = true;
				}
			}
			if (hasAppearance == false)
			{
				return false;
			}
		}
		return true;
	}

	std::map<std::string, IsmPtr> CombinatorialTrainer::learnFullyMeshedTopologyPerPattern(bool naive)
	{
		LogHelper::logLine();
		std::map<std::string, IsmPtr> fullyMeshedTopologyPerPattern;
        for (std::string& patternNameIt : patternNames)
		{
			LogHelper::logMessage("Learning fully meshed topology for pattern " + patternNameIt);
			TreePtr fullyMeshedTopology = TreePtr(new Tree(patternNameIt,
						allObjectRelationsPerPattern.at(patternNameIt), naive));
			fullyMeshedTopologyPerPattern[patternNameIt] = fullyMeshedTopology->getISM();;
		}

		LogHelper::logMessage("Learning is done");
		LogHelper::logLine();

		return fullyMeshedTopologyPerPattern;
	}

	void CombinatorialTrainer::storeTestSetsToDB(PatternNameToObjectSet testSet,
			const path dbFilePath,
			const std::string & type)
	{
		if(!boost::filesystem::exists(dbFilePath.parent_path()))
			boost::filesystem::create_directories(dbFilePath.parent_path());
		try
		{
			TableHelperPtr localTableHelper(new TableHelper(dbFilePath.string()));
			localTableHelper->dropTables();
			localTableHelper->createTablesIfNecessary();
			for (PatternNameToObjectSet::iterator testSetIt = testSet.begin();
					testSetIt != testSet.end();
					++testSetIt)
			{
				LogHelper::logMessage("Storing the " + type + " object sets for pattern "
						+ testSetIt->first + " to " + dbFilePath.string());
				LogHelper::displayProgress(0);
				localTableHelper->insertRecordedPattern(testSetIt->first);
				std::vector<ISM::ObjectSetPtr> objectSets = testSetIt->second;
				for (unsigned int i = 0; i < objectSets.size(); ++i) {
					LogHelper::displayProgress(((double) i + 1) / objectSets.size());
					localTableHelper->insertRecordedObjectSet(objectSets[i], testSetIt->first);
				}
				LogHelper::displayProgress(1);
				std::cout << std::endl;

			}
		} catch (soci::soci_error& e)
		{
			LogHelper::logMessage("Probablay the filepath " + dbFilePath.string()
					+ " used to store evaluation results in storeISMToDB in CombinatorialTrainer and"
					+ " setTestSets in Tester does not exist on your system", LOG_ERROR);
			std::cerr << "soci error\n" << e.what() << std::endl;
		}
		LogHelper::logMessage("");
	}

	PatternNameToObjectSet CombinatorialTrainer::loadTestSetsFromDB(std::string fileName)
	{
		path dbfilename = fileName;
		TableHelperPtr localTableHelper(new TableHelper(dbfilename.string()));
		std::vector<std::string> patternNames = tableHelper->getRecordedPatternNames();
		PatternNameToObjectSet testSet;

		for (unsigned int i = 0; i < patternNames.size(); ++i)
		{
			testSet[patternNames[i]]  = localTableHelper->getRecordedPattern(patternNames[i])->objectSets;
			LogHelper::logMessage("Loaded " + std::to_string(testSet[patternNames[i]].size()) +
					" test sets for pattern " +  patternNames[i] + " from DB " + fileName);
		}

		return testSet;
	}

	std::pair<PatternNameToObjectSet, PatternNameToObjectSet> CombinatorialTrainer::createTestSets(
			double binSize, double maxAngleDeviation, double confidenceThreshold, unsigned int testSetCount)
	{
		PatternNameToObjectSet validTestSetsPerPattern;
		PatternNameToObjectSet invalidTestSetsPerPattern;

		RecognizerPtr recognizer = ISM::RecognizerPtr(new ISM::Recognizer("", binSize, maxAngleDeviation));

		ObjectSetValidatorPtr objectSetValidator = 
			ObjectSetValidatorPtr(new ObjectSetValidator(recognizer, confidenceThreshold));

		TestSetGeneratorPtr testSetGenerator =
			TestSetGeneratorPtr(new TestSetGenerator(allObjectRelationsPerPattern, objectSetValidator));

		for (std::map<std::string, ISM::TracksPtr>::iterator patternIt = this->objectTracksPerPattern.begin();
				patternIt != this->objectTracksPerPattern.end();
				++patternIt)
		{
			std::pair<std::vector<ObjectSetPtr>, std::vector<ObjectSetPtr>> testSets =
				testSetGenerator->generateTestSets(patternIt->first, patternIt->second,
						mFullyMeshedTopologyPerPattern.at(patternIt->first), testSetCount);

			validTestSetsPerPattern[patternIt->first] = testSets.first;
			invalidTestSetsPerPattern[patternIt->first] = testSets.second;
		}

		return std::make_pair(validTestSetsPerPattern, invalidTestSetsPerPattern);
	}

	void CombinatorialTrainer::initTestSets(double binSize, double maxAngleDeviation,
			double confidenceThreshold, std::string loadValidTestSetsFrom,
			std::string loadInvalidTestSetsFrom, unsigned int testSetCount)
	{
		PatternNameToObjectSet validTestSetsPerPattern;
		PatternNameToObjectSet invalidTestSetsPerPattern;

		bool createInvalidTestSet = true;
		bool createValidTestSet = true;

		if (loadValidTestSetsFrom.compare("") != 0) {
			createValidTestSet = false;
			validTestSetsPerPattern = loadTestSetsFromDB(loadValidTestSetsFrom);
		}

		if (loadInvalidTestSetsFrom.compare("") != 0) {
			createInvalidTestSet = false;
			invalidTestSetsPerPattern = loadTestSetsFromDB(loadInvalidTestSetsFrom);
		}

		if (createInvalidTestSet || createValidTestSet)
		{
			std::pair<PatternNameToObjectSet, PatternNameToObjectSet> testSets =
				createTestSets(binSize, maxAngleDeviation, confidenceThreshold, testSetCount);

			if (createValidTestSet) validTestSetsPerPattern = testSets.first;
			if (createInvalidTestSet) invalidTestSetsPerPattern = testSets.second;
		}

		LogHelper::logLine();

		mTestSets = std::make_pair(validTestSetsPerPattern, invalidTestSetsPerPattern);
	}

	std::string CombinatorialTrainer::genTimeString()
	{
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer, 80,"%d-%b-%Y_%H:%M:%S", timeinfo);
		return std::string(buffer);
	}

	void CombinatorialTrainer::initStartTopologiesPerPattern(std::string loadStartTopologiesFrom)
	{
		if (loadStartTopologiesFrom.compare("") != 0)
		{
			path from = path(loadStartTopologiesFrom);
			if (!boost::filesystem::exists(from))
			{
				std::stringstream ss;
				ss << loadStartTopologiesFrom << " does not exist!";
				throw std::runtime_error(ss.str());
			}

			std::string line;
			std::ifstream file;
			file.open(from.string());

			while (std::getline(file, line))
			{
				std::vector<std::string> tokens;
				boost::split(tokens, line, boost::is_any_of(":"));

				std::string patternName = tokens[0];
				ObjectRelations allObjectRelations = allObjectRelationsPerPattern[patternName];
				ObjectRelations objectRelations;

				std::vector<bool> bitvector(allObjectRelations.size(), 0) ;

				for (unsigned int i = 1; i < tokens.size(); ++i)
				{
					unsigned int index = boost::lexical_cast<unsigned int>(tokens[i]);
					objectRelations[index] = allObjectRelations[index];
					bitvector[index] = 1;
				}

				std::string identifier = "";
				for (unsigned int i = 0; i < bitvector.size(); ++i)
				{
					identifier += bitvector[i] ? "1" : "0";
				}

				if (mStartTopologiesPerPattern.find(patternName) == mStartTopologiesPerPattern.end())
				{
					mStartTopologiesPerPattern[patternName] = std::vector<TopologyPtr>();
				}
				TopologyPtr topology = TopologyPtr(new Topology());
				topology->objectRelations = objectRelations;
				topology->identifier = identifier;
				mStartTopologiesPerPattern[patternName].push_back(topology);
			}
		}
	}

}
