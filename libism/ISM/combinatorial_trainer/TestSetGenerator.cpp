/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "TestSetGenerator.hpp"

#include <ctime>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <sys/wait.h>
#include <thread>
#include <sys/time.h>
#include <string>
#include <chrono>
#include <random>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/thread.hpp>
#include <boost/functional/hash.hpp>
#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

#include "utility/Util.hpp"
#include "utility/GeometryHelper.hpp"
#include "ImplicitShapeModel.hpp"

namespace ISM {

	std::pair<std::vector<ObjectSetPtr>, std::vector<ObjectSetPtr>> TestSetGenerator::generateTestSets(
			const std::string &patternName, const TracksPtr& tracks, const IsmPtr ism, unsigned int testSetCount)
	{
		LogHelper::logMessage("Generating " + std::to_string(testSetCount) + " test sets for pattern " + patternName + ":");

		std::vector<ObjectSetPtr> validTestSets;
		std::vector<ObjectSetPtr> invalidTestSets;

		mObjectSetValidator->setISM(ism);

		std::random_device rd;
		ENG  eng;
		eng.seed(rd());
		DIST dist(0,RAND_MAX);
		GEN  gen(eng,dist);

		LogHelper::displayProgress(0);

		for (unsigned int i = 0; i < testSetCount; ++i)
		{
			ObjectSetPtr randomObjectSet = generateRandomObjectSetFromTracks(tracks, patternName, gen);

			if (mObjectSetValidator->isSetValid(randomObjectSet, patternName).first)
			{
				validTestSets.push_back(randomObjectSet);
			}
			else
			{
				invalidTestSets.push_back(randomObjectSet);
			}
			LogHelper::displayProgress(((double) i) / testSetCount);
		}

		LogHelper::displayProgress(1);

		LogHelper::logMessage("\nThe test sets for pattern " + patternName
				+ " contain " + std::to_string(validTestSets.size())
				+ " valid and " + std::to_string(invalidTestSets.size())
				+ " invalid object sets.\n");

		return std::make_pair(validTestSets, invalidTestSets);
	}


	ObjectSetPtr TestSetGenerator::generateRandomObjectSetFromTracks(const TracksPtr& allTracks,
			const std::string& pattern, GEN &gen)
	{
		ObjectSetPtr randomObjectSet(new ObjectSet());
		ObjectPtr referenceObject;

		unsigned int tries = 0;
		const unsigned int maxTries = (allTracks->tracks.size() + allTracks->tracks[0]->objects.size()) * 100;
		while(!referenceObject && tries < maxTries)
		{
			unsigned randomRefTrack = gen() % allTracks->tracks.size();
			unsigned randomRefPos = gen() % allTracks->tracks[randomRefTrack]->objects.size();
			referenceObject = allTracks->tracks[randomRefTrack]->objects[randomRefPos];
			++tries;
		}
		if(!referenceObject)
		{
			std::cerr << "TestSetGenerator::generateTestSetsIterative couldnt find referenceObject. Increase maxTries?" << std::endl;
			throw std::runtime_error("TestSetGenerator::generateTestSetsIterative couldnt find referenceObject. Increase maxTries?");
		}

		randomObjectSet->insert(referenceObject);

		for (std::vector<TrackPtr>::iterator track = allTracks->tracks.begin(); track != allTracks->tracks.end(); ++track)
		{
			if ((*track)->type == referenceObject->type && (*track)->observedId == referenceObject->observedId)
			{
				continue;
			}

			const ObjectRelations& allRelations = this->mAllObjectRelationsPerPattern.at(pattern);
			for (const std::pair<unsigned int, ObjectRelationPtr>& relation : allRelations)
			{
				if ( (relation.second->getVotesFromAForReferencePoseB().size() > 0
							|| relation.second->getVotesFromBForReferencePoseA().size() > 0)
						&& relation.second->containsObject(referenceObject->type, referenceObject->observedId) == true
						&& relation.second->containsObject((*track)->type, (*track)->observedId) == true)
				{
					PosePtr randomRelativePose;

					//if objects dont occur together size = 0
					if (relation.second->getVotesFromBForReferencePoseA().size() > 0 &&
							relation.second->getObjectTypeA() == referenceObject->type &&
							relation.second->getObjectIdA() == referenceObject->observedId)
					{
						unsigned randomTimeStep = gen() % relation.second->getVotesFromBForReferencePoseA().size();
						VoteSpecifierPtr randomVote = relation.second->getVotesFromBForReferencePoseA()[randomTimeStep];
						PointPtr randomRelativePoint = GeometryHelper::getSourcePoint(referenceObject->pose, randomVote->refToObjectQuat, randomVote->radius);
						randomRelativePose = GeometryHelper::getSourcePose(referenceObject->pose, randomRelativePoint, randomVote->refToObjectPoseQuat);
					}
					else if(relation.second->getVotesFromAForReferencePoseB().size() > 0)
					{
						unsigned randomTimeStep = gen() % relation.second->getVotesFromAForReferencePoseB().size();
						VoteSpecifierPtr randomVote = relation.second->getVotesFromAForReferencePoseB()[randomTimeStep];
						PointPtr randomRelativePoint = GeometryHelper::getSourcePoint(referenceObject->pose, randomVote->refToObjectQuat, randomVote->radius);
						randomRelativePose = GeometryHelper::getSourcePose(referenceObject->pose, randomRelativePoint, randomVote->refToObjectPoseQuat);
					}
					ObjectPtr fittingObject(new Object((*track)->type, randomRelativePose, (*track)->observedId));
					randomObjectSet->insert(fittingObject);
					break;
				}
			}
		}

		return randomObjectSet;
	}
}
