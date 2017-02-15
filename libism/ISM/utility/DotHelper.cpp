/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "DotHelper.hpp"

#include <set>
#include <fstream>
#include <iostream>
#include <math.h>

namespace ISM {

	void DotHelper::storeISMToDot(const path outputPath, const std::string& patternName,
			const std::string& topologyIdentifier,
			const std::map<std::string, std::vector<ISM::VoteSpecifierPtr>>& objectDefinitons)
	{
		std::stringstream fileName;
		fileName << topologyIdentifier << "_ISM.dot";
		path filePath = outputPath / fileName.str();

		std::vector<std::pair<std::string, std::string> > alreadyTakenCombinations;
		std::ofstream file;
		std::ios_base::iostate exceptionMask = file.exceptions() |
			std::ios::failbit |
			std::ios::badbit;
		file.exceptions(exceptionMask);
		try
		{
			file.open(filePath.string());
			file << "digraph " << topologyIdentifier << " {\n";

			std::set<std::string> patternNames;
            for (const std::pair< std::string, std::vector<ISM::VoteSpecifierPtr>>& typeIt : objectDefinitons)
			{
                for (const VoteSpecifierPtr& voteIt : typeIt.second)
				{
					if (std::find(alreadyTakenCombinations.begin(),
								alreadyTakenCombinations.end(),
								std::make_pair(voteIt->patternName, voteIt->objectType))
							== alreadyTakenCombinations.end())
					{
						file << voteIt->patternName << " -> " << voteIt->objectType << "[dir=\"back\"];\n";

						alreadyTakenCombinations.push_back(std::make_pair(voteIt->patternName,
									voteIt->objectType));

						patternNames.insert(voteIt->patternName);
					}
				}
			}

			for (std::string name : patternNames)
			{
				file << name << "[shape=\"box\"];\n";
			}

            for (TrackPtr track : mTracksPerPattern[patternName]->tracks)
			{
				file << track->type << ";\n";
			}

			file << "}\n\n";
			file.flush();
			file.close();
		}
		catch (std::ios_base::failure& e)
		{
			std::cerr << e.what() << "\n";
		}
	}

	void DotHelper::storeRelationsToDot(const path outputPath, const std::string& patternName,
			const std::string& topologyIdentifier,
			const ObjectRelations& relations,
			ObjectRelations referenceRelations)
	{
		std::stringstream fileName;
		fileName << topologyIdentifier << "_Relations.dot";
		path filePath = outputPath / fileName.str();

		std::ofstream file;
		std::ios_base::iostate exceptionMask = file.exceptions() |
			std::ios::failbit |
			std::ios::badbit;
		file.exceptions(exceptionMask);

		try
		{
			file.open(filePath.string());
			file << "graph " << topologyIdentifier << " {\n";

			if (referenceRelations.empty())
			{
				//If the reference topology has no relations, we don't check for removed or added relations.
                for (const std::pair<unsigned int, ObjectRelationPtr>& relation : relations)
				{
					file << relation.second->getObjectTypeA() << " -- "
						<< relation.second->getObjectTypeB() << ";\n";
				}
			}
			else
			{
				ObjectRelations addedRelations;
				ObjectRelations removedRelations;
				ObjectRelations unchangedRelations;

				//Find the relations that were changed compared to the reference topology.
                for (const std::pair<unsigned int, ObjectRelationPtr>& relation : relations)
				{
					if (referenceRelations.find(relation.first) == referenceRelations.end())
					{
						addedRelations.insert(relation);
					}
					else
					{
						unchangedRelations.insert(relation);
					}

					referenceRelations.erase(relation.first);
				}

				removedRelations = referenceRelations;

				for (auto relation : unchangedRelations)
				{
					file << relation.second->getObjectTypeA() << " -- "
						<< relation.second->getObjectTypeB() << ";\n";
				}

				for (auto relation : addedRelations)
				{
					file << relation.second->getObjectTypeA() << " -- "
						<< relation.second->getObjectTypeB() << "[color=\"green\", penwidth=\"2\"];\n";
				}

				for (auto relation : removedRelations)
				{
					file << relation.second->getObjectTypeA() << " -- "
						<< relation.second->getObjectTypeB() << "[color=\"red\", style=\"dotted\", penwidth=\"2\"];\n";
				}
			}

			//Arange the nodes in a circle
			double angle = 0;
			double angleStepSize = 2 * M_PI / mTracksPerPattern[patternName]->tracks.size();

			//Calculate the radius of the circle so that neighbouring nodes have a distance equal to NODE_DISTANCE
			double radius = sqrt((NODE_DISTANCE * NODE_DISTANCE) /
					(sin(angleStepSize) * sin(angleStepSize) + cos(angleStepSize) * cos(angleStepSize)));

            for (TrackPtr& track : mTracksPerPattern[patternName]->tracks)
			{
				//Calculate x and y position values and round result to four places
				double x = radius * floor(sin(angle) * 1000) / 1000;
				double y = radius * floor(cos(angle) * 1000) / 1000;

				file << track->type << " [pos=\"" << x << "," << y << "!\"];\n";
				angle += angleStepSize;
			}

			file << "}\n\n";
			file.flush();
			file.close();
		}
		catch (std::ios_base::failure& e)
		{
			std::cerr << e.what() << "\n";
		}
	}

}
