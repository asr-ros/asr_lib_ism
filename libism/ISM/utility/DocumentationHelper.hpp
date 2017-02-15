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

#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>

#include "../combinatorial_optimization/CostFunction.hpp"
#include "../combinatorial_trainer/CombinatorialTrainerParameters.hpp"
#include "../combinatorial_trainer/Topology.hpp"
#include "../combinatorial_trainer/EvaluationResult.hpp"

#include "SVGHelper.hpp"
#include "DotHelper.hpp"

namespace ISM
{
using boost::filesystem::path;

class DocumentationHelper
{
	public:
		DocumentationHelper(path outputPath, path dbPath,
                std::map<std::string, ISM::TracksPtr>& tracksPerPattern) :
            mOutputPath(outputPath), mSourceDBPath(dbPath)
		{
			boost::filesystem::create_directories(mOutputPath);
			boost::filesystem::create_directories(mOutputPath / SVG_SUBFOLDER_NAME);
			boost::filesystem::create_directories(mOutputPath / CSV_SUBFOLDER_NAME);
			boost::filesystem::create_directories(mOutputPath / TOPOLOGIES_SUBFOLDER_NAME);

			mSVGHelper = SVGHelperPtr(new SVGHelper(mOutputPath / SVG_SUBFOLDER_NAME));
			mDotHelper = DotHelperPtr(new DotHelper(tracksPerPattern));
		}

		void storeOptimizationRun(std::vector<std::vector<std::pair<TopologyPtr, unsigned int>>>& history,
				CostFunctionPtr<TopologyPtr> globalCostFunction,
				double elapsedRuntime, const std::string& patternName);

		void storeTopology(TopologyPtr toStore, const std::string& patternName,
				const std::string& topologyName,
				std::map<std::string, std::vector<ISM::VoteSpecifierPtr>> objectDefinitons);

		void storeIsm(const std::string& filename, const IsmPtr& ism);
		void storeIsm(const IsmPtr& ism);
		void writeResult();
		void setReferenceTopology(TopologyPtr topology, const std::string & patternName);

	private:
		const std::string SVG_SUBFOLDER_NAME = "SVG";
		const std::string CSV_SUBFOLDER_NAME = "CSV";
		const std::string TOPOLOGIES_SUBFOLDER_NAME = "Topologies";
		const std::string SELECTED_TOPOLOGIES_FOLDER = "allSelectedTopologies";

		path mOutputPath;
		path mSourceDBPath;

		SVGHelperPtr mSVGHelper;
		DotHelperPtr mDotHelper;

		bool mStoreISM = true;

		unsigned int mSelectedTopologyCounter = 1;

		TopologyPtr mReferenceTopology;

		void storeTopologiesToCSV(std::vector<TopologyPtr> selectedTopologies,
				const std::string& patternNames);

		void storeMetadataToCSV(unsigned int numEvaluatedTopologies,
				unsigned int numOptimizationRounds, double elapsedRuntime,
				const std::string& patternName);

		void writeToFile(path filePath, const std::string& content);
        void writeIsmToDB(path dbPath, const IsmPtr & ism);


}; typedef boost::shared_ptr<DocumentationHelper> DocumentationHelperPtr;

}

