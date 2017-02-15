/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include <utility/TableHelper.hpp>
#include <utility/DocumentationHelper.hpp>
#include <common_type/ObjectSet.hpp>
#include <common_type/Tracks.hpp>
#include <common_type/VoteSpecifier.hpp>
#include <common_type/Pattern.hpp>
#include <fstream>
#include <iostream>
#include <string>


#include "common_type/RecognitionResult.hpp"
#include "ObjectRelation.hpp"
#include "ImplicitShapeModel.hpp"
#include "TestSetGenerator.hpp"

#include "TopologyValidationResultContainer.hpp"
#include "Tester.hpp"

#include "CombinatorialTrainerParameters.hpp"
#include "Topology.hpp"
#include "../typedef.hpp"

namespace ISM
{
	using boost::filesystem::path;
	/**
	*CombinatorialTrainer class. In opposite to the "Trainer" class, this class build a scene model depending only on the
	*relevant relations between objects in the scene
	*/
class CombinatorialTrainer
{
	public:
		CombinatorialTrainer(CombinatorialTrainerParameters params);

		std::map<std::string, std::pair<double, TreePtr> > learn();
	private:
		TableHelperPtr tableHelper;

		std::vector<std::string> patternNames;
		//These are the Tracks of ALL patterns in the DB file
		std::map<std::string, ISM::TracksPtr> objectTracksPerPattern;
		//These are all objectRelations (a fully connected topology) of each Pattern in the DB file
		std::map<std::string, ISM::ObjectRelations> allObjectRelationsPerPattern;
		//contains all relations of objects to themselves
		std::map<std::string, std::vector<ISM::ObjectRelationPtr> > allSelfRelationsPerPattern;
		//filepath to the objects models for visualization
		std::map<std::string, std::map<std::string, boost::filesystem::path> > objectModelsPerPattern;
		path mOutputDataPath;
		path mDBPath;

		DocumentationHelperPtr mDocumentationHelper;

		std::map<std::string, IsmPtr> mFullyMeshedTopologyPerPattern;
		std::map<std::string, std::vector<TopologyPtr>> mStartTopologiesPerPattern;

		std::pair<PatternNameToObjectSet, PatternNameToObjectSet> mTestSets;

		CombinatorialTrainerParameters mParams;

		//Get all Tracks of all pattern in the DB file
		//I.e: Get a table where each entry marks all tracks of all objects in one distinct pattern
		const std::map<std::string, ISM::TracksPtr> getRecordedObjectsTracks();
		//Calculate all relations between all objects in the same pattern, for all patterns in the DB file
		const std::map<std::string, ISM::ObjectRelations> calculateAllObjectRelations();
		bool containsAllObjects(const ISM::ObjectRelations& topology, const std::string& patternName) const;
		void drawISM(unsigned ismId, std::string patternName, std::map<std::string, std::vector<VoteSpecifierPtr> > voteSpecifiersPerObject);
		TopologyValidationResultsPtr getValidationResultsForPattern(std::string patternName);

		std::map<std::string, IsmPtr> learnFullyMeshedTopologyPerPattern(bool naive = false);
		bool checkCorrectOrder(std::map<std::string, ISM::ObjectRelations> allObjectRelationsPerPattern);
		void storePatternToDB(std::map<std::string, IsmPtr> bestISMPerPattern);

		void storeTestSetsToDB(PatternNameToObjectSet testSet,
				const path dbFilePath,
				const std::string & type);

		PatternNameToObjectSet loadTestSetsFromDB(std::string fileName);

		std::pair<PatternNameToObjectSet, PatternNameToObjectSet> createTestSets(double binSize, double maxAngleDeviation,
				double confidenceThreshold, unsigned int testSetCount);

		void initTestSets(double binSize, double maxAngleDeviation, double confidenceThreshold,
				std::string loadValidTestSetsFrom, std::string loadInvalidTestSetsFrom, unsigned int testSetCount);

		void initStartTopologiesPerPattern(std::string loadStartTopologiesFrom);
		std::string genTimeString();

}; typedef boost::shared_ptr<CombinatorialTrainer> CombinatorialTrainerPtr;

}
