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

#include "limits"

#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "../combinatorial_trainer/ObjectRelation.hpp"
#include "../combinatorial_trainer/EvaluationResult.hpp"
#include "../combinatorial_trainer/Topology.hpp"
#include "../combinatorial_trainer/EvaluationResult.hpp"
#include "../combinatorial_optimization/CostFunction.hpp"

namespace ISM
{
using boost::filesystem::path;
using boost::property_tree::ptree;


class SVGHelper
{
	public:
		SVGHelper(path outputPath) : mOutputPath(outputPath) {}

		void writeResult();

		void processHistory(std::vector<std::vector<std::pair<TopologyPtr, unsigned int>>>& history,
			CostFunctionPtr<TopologyPtr> globalCostFunction, const std::string& patternName);

	private:
		struct TopologyContainer {
			EvaluationResult er;

			std::string identifier;
			unsigned int index;
			double cost;

			int y = 0;
			int x = 0;
		};

		struct OptimizationRound {
			std::vector<TopologyContainer> evaluatedTopologies;
			int selectedTopologyIndex = -1;
			unsigned int selectedTopologyType = 0;
		};

		struct OptimizationRun {
			std::vector<OptimizationRound> allRounds;

			double minCost;
			double maxCost;

			double minAverageRecognitionRuntime;
			double maxAverageRecognitionRuntime;

			int lastSelectedIndex = -1;
		};

		const std::string ATR_CHOSEN_INDEX = "chosen_index";
		const std::string ATR_ROUND_NUMBER = "round_number";
		const std::string ATR_EVALUATION_RESULT = "evaluation_result";
		const std::string ATR_PATTERN_NAME = "pattern_name";
		const std::string ATR_RELATION_IDS = "relation_ids";

		const std::string NAME_TOPOLOGY = "topology";
		const std::string NAME_ROUND = "round";
		const std::string NAME_OPTIMIZATION_RUN = "optimization-run";

		const std::string COLOR_FIRST_HIGHLIGHT = "aqua";
		const std::string COLOR_LAST_HIGHLIGHT = "blue";
		const std::string COLOR_HIGHLIGHT = "cornflowerblue";
		const std::string COLOR_BEST = "purple";

		const std::string COLOR_RANDOM_WALK_HIGHLIGHT = "black";
		const std::string COLOR_RANDOM_RESTART_HIGHLIGHT = "black";

		const unsigned int MIN_CIRCLE_RADIUS = 100;
		const unsigned int MAX_CIRCLE_RADIUS = 160;
		const unsigned int PADDING = 20;
		const unsigned int FONT_SIZE = 30;
		const unsigned int HIGHLIGHT_WIDTH = 7;
		const unsigned int LINE_WIDTH = 6;

		const unsigned int MAX_TOPOLOGIES_PER_LINE = 12;

		path mOutputPath;

		OptimizationRound mCurrentOptimizationRound;
		OptimizationRun mCurrentOptimizationRun;
		std::string mCurrentPatternName;
		std::vector<OptimizationRound> mAllEvaluationRounds;
		std::map<std::string, std::vector<OptimizationRun>> mPatternNameToOptimizationRuns;

		std::vector<ptree> createSVG(std::vector<OptimizationRun> optimizationRuns);
		ptree genCircleSVG(double radius, int x, int y, const std::string& circleColor,
				bool highlight, const std::string& highlightColor);
		ptree genPolylineSVG(std::vector<std::string> linePoints);
		ptree genTextSVG(const std::string& lineOne, const std::string& lineTwo,
				const std::string& lineThree, int x, int y);
		ptree genRectSVG(int x, int y, unsigned int width,
				unsigned int height, const std::string& colorString);
		ptree genSeparatorLineSVG(int y, int x1, int x2);

		ptree createXML(const std::string& patternName);
		ptree genTopologyContainerXML(TopologyContainer tc);
		ptree genEvaluationRoundXML(unsigned int roundNumber, unsigned int chosenIndex);

		std::string genRGBString(double cost, double minCost, double maxCost);

		double calculateCircleRadius(double evaluationDuration,
				double minEvaluationDuration,
				double maxEvaluationDuration);

		std::vector<int> calculateXValues(unsigned int numTopologies, unsigned int imageWidth);

		void calculatePositions(std::vector<OptimizationRun>& optimizationRuns,
				std::vector<std::vector<int>>& seperatorLinePositions,
				unsigned int& imgWidth, unsigned int& imgHeight);

}; typedef boost::shared_ptr<SVGHelper> SVGHelperPtr;

}
