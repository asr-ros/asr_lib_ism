/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "SVGHelper.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include <boost/version.hpp>

namespace ISM
{

	void SVGHelper::processHistory(std::vector<std::vector<std::pair<TopologyPtr, unsigned int>>>& history,
			CostFunctionPtr<TopologyPtr> globalCostFunction, const std::string& patternName)
	{
		double minAverageRecognitionRuntime = std::numeric_limits<double>::max();
		double maxAverageRecognitionRuntime = std::numeric_limits<double>::min();

		double minCost = std::numeric_limits<double>::max();
		double maxCost = std::numeric_limits<double>::min();

		OptimizationRun optRun;
		optRun.lastSelectedIndex = 0;

		for (unsigned int i = 0; i < history.size(); ++i)
		{
			OptimizationRound optRound;
			for (unsigned int j = 0; j < history[i].size(); ++j)
			{
				if (history[i][j].second != 0)
				{
					optRound.selectedTopologyIndex = j;
					optRound.selectedTopologyType = history[i][j].second;

					optRun.lastSelectedIndex = i;
				}

				TopologyPtr topology = history[i][j].first;

				TopologyContainer tc;
				tc.er = topology->evaluationResult;
				tc.index = topology->index;
				tc.identifier = topology->identifier;

				double cost = globalCostFunction->calculateCost(topology);
				cost = cost == std::numeric_limits<double>::max() ? -1 : cost;
				tc.cost = cost;

				minCost = std::min(minCost, cost);
				maxCost = std::max(maxCost, cost == std::numeric_limits<double>::infinity() ? 0 : cost);

				optRound.evaluatedTopologies.push_back(tc);

				minAverageRecognitionRuntime = std::min(minAverageRecognitionRuntime, tc.er.averageRecognitionRuntime);
				maxAverageRecognitionRuntime = std::max(maxAverageRecognitionRuntime, tc.er.averageRecognitionRuntime);
			}
			optRun.allRounds.push_back(optRound);
		}

		optRun.minAverageRecognitionRuntime = minAverageRecognitionRuntime;
		optRun.maxAverageRecognitionRuntime = maxAverageRecognitionRuntime;

		optRun.minCost = minCost;
		optRun.maxCost = maxCost;

		mPatternNameToOptimizationRuns[patternName].push_back(optRun);
	}

	void SVGHelper::writeResult()
	{
        #if (BOOST_VERSION >= 105600)
            boost::property_tree::xml_writer_settings<std::string> settings('\t', 1);
        #else
            boost::property_tree::xml_writer_settings<char> settings('\t', 1);
        #endif

		for (std::map<std::string, std::vector<OptimizationRun>>::iterator it = mPatternNameToOptimizationRuns.begin();
				it != mPatternNameToOptimizationRuns.end();
				++it)
		{
			std::vector<ptree> roots = createSVG(it->second);
			for (unsigned int i = 0; i < roots.size(); ++i)
			{
				std::string svgFileName = mOutputPath.string() + "/visulisation- "
                    + it->first + "_" + std::to_string(i);
                boost::property_tree::write_xml(svgFileName + ".svg", roots[i], std::locale(), settings);
			}
		}
	}

	std::vector<int> SVGHelper::calculateXValues(unsigned int numTopologies, unsigned int imageWidth)
	{
		std::vector<int> xValues;
		unsigned int center = imageWidth / 2;

		for (unsigned int i = 0; i < numTopologies;)
		{
			unsigned int numCirclesThisLine = std::min(MAX_TOPOLOGIES_PER_LINE, numTopologies - i);
			unsigned int half;
			int x;

			if (numCirclesThisLine % 2 == 0)
			{
				half = numCirclesThisLine / 2;
				x = center - (PADDING / 2 + MAX_CIRCLE_RADIUS + (half - 1)
						* (PADDING + 2 * MAX_CIRCLE_RADIUS));
			} 
			else 
			{
				half = (numCirclesThisLine - 1) / 2;
				x = center - half * (PADDING + 2 * MAX_CIRCLE_RADIUS);
			}

			for (unsigned int j = 0; j < numCirclesThisLine; j++)
			{
				xValues.push_back(x);
				x += PADDING + 2 * MAX_CIRCLE_RADIUS;
			}
			i += numCirclesThisLine;
		}

		return xValues;
	}

	void SVGHelper::calculatePositions(std::vector<OptimizationRun>& optimizationRuns,
			std::vector<std::vector<int>>& seperatorLinePositions,
			unsigned int& imgWidth, unsigned int& imgHeight)
	{
		unsigned int numStartingTopologies = 0;
		unsigned int maxRoundSize = 0;

		for (unsigned int i = 0; i < optimizationRuns.size(); ++i)
		{
			OptimizationRun optRun = optimizationRuns[i];
			numStartingTopologies += optRun.allRounds[0].evaluatedTopologies.size();
			for (unsigned int j = 0; j < optRun.allRounds.size(); ++j)
			{
				maxRoundSize = std::max(maxRoundSize,
						(unsigned int) optRun.allRounds[j].evaluatedTopologies.size());
			}
		}

		unsigned int maxNumTopologiesPerLine = std::min(MAX_TOPOLOGIES_PER_LINE,
				std::max(maxRoundSize, numStartingTopologies));

		imgWidth = maxNumTopologiesPerLine * 2 * MAX_CIRCLE_RADIUS + (maxNumTopologiesPerLine + 2)
			* PADDING;

		std::vector<int> startingXValues = calculateXValues(numStartingTopologies, imgWidth);
		unsigned int startingXValuesIndex = 0;

		unsigned int x;
		unsigned int y;
		unsigned int maxY = 0;

		for (unsigned int i = 0; i < optimizationRuns.size(); ++i)
		{
			OptimizationRun & optRun = optimizationRuns[i];
			seperatorLinePositions.push_back(std::vector<int>());
			y = PADDING + MAX_CIRCLE_RADIUS;
			unsigned int index = 0;

			for (unsigned int j = 0; j < optRun.allRounds.size(); ++j)
			{
				index = 0;
				std::vector<int> xValues;
				if (j != 0)
				{
					xValues = calculateXValues(optRun.allRounds[j].evaluatedTopologies.size(),
							imgWidth);
				}

				for (unsigned int k = 0; k < optRun.allRounds[j].evaluatedTopologies.size(); ++k)
				{
					if (index == MAX_TOPOLOGIES_PER_LINE)
					{
						y += PADDING + MAX_CIRCLE_RADIUS * 2;
						index = 0;
					}

					if (j == 0)
					{
						x = startingXValues[startingXValuesIndex];
						startingXValuesIndex++;
					} 
					else 
					{
						x = xValues[k];
					}

					TopologyContainer & tc = optRun.allRounds[j].evaluatedTopologies[k];
					tc.x = x;
					tc.y = y;
					index++;
				}

				if (j != optimizationRuns[i].allRounds.size() - 1)
				{
					seperatorLinePositions[i].push_back(y + MAX_CIRCLE_RADIUS + 2 * PADDING);
					y += (2 * PADDING + MAX_CIRCLE_RADIUS) * 2;
				} 
				else
				{
					y += PADDING + MAX_CIRCLE_RADIUS * 2;
				}
			}
			maxY = std::max(y, maxY);
		}

		imgHeight = maxY - MAX_CIRCLE_RADIUS;
	}

	std::vector<ptree> SVGHelper::createSVG(std::vector<OptimizationRun> optimizationRuns)
	{
		std::vector<ptree> roots;

		std::vector<std::vector<int>> seperatorLinePositions;
		unsigned int imgWidth, imgHeight;
		calculatePositions(optimizationRuns, seperatorLinePositions, imgWidth, imgHeight);

		double maxAverageRecognitionRuntime = std::numeric_limits<double>::min();
		double minAverageRecognitionRuntime = std::numeric_limits<double>::max();

		for (unsigned int i = 0; i < optimizationRuns.size(); ++i)
		{
			OptimizationRun optRun = optimizationRuns[i];

			maxAverageRecognitionRuntime = std::max(maxAverageRecognitionRuntime, optRun.maxAverageRecognitionRuntime);
			minAverageRecognitionRuntime = std::min(minAverageRecognitionRuntime, optRun.minAverageRecognitionRuntime);
		}

		for (unsigned int i = 0; i < optimizationRuns.size(); ++i)
		{
			OptimizationRun optRun = optimizationRuns[i];
			std::vector<std::string> linePoints;
			std::vector<ptree> circleNodes;
			std::vector<ptree> textNodes;
			unsigned int index = 0;

			for (unsigned int j = 0; j < optRun.allRounds.size(); ++j)
			{
				OptimizationRound optRound = optRun.allRounds[j];

				for (unsigned int k = 0; k < optRound.evaluatedTopologies.size(); ++k) {
					TopologyContainer tc = optRound.evaluatedTopologies[k];
					int x = tc.x;
					int y = imgHeight - tc.y;
					std::string highlightColor = "";

					bool foundChosen = (int) k == optRound.selectedTopologyIndex;
					if (foundChosen)
					{
						linePoints.push_back(std::to_string(x) + "," + std::to_string(y));
						if (j == 0)
						{
							highlightColor = COLOR_FIRST_HIGHLIGHT;
						} 
						else if ((int) j == optRun.lastSelectedIndex) 
						{
							highlightColor = COLOR_LAST_HIGHLIGHT;
						} 
						else 
						{
							switch (optRound.selectedTopologyType) {
								case 1:
									highlightColor = COLOR_HIGHLIGHT;
									break;
								case 2:
									highlightColor = COLOR_RANDOM_RESTART_HIGHLIGHT;
									break;
								case 3:
									highlightColor = COLOR_RANDOM_WALK_HIGHLIGHT;
									break;
								case 4:
									highlightColor = COLOR_BEST;
									break;
								default:
									highlightColor = COLOR_HIGHLIGHT;
									break;
							}
						}
					}

					circleNodes.push_back(genCircleSVG(
								calculateCircleRadius(tc.er.averageRecognitionRuntime,
									minAverageRecognitionRuntime,
									maxAverageRecognitionRuntime),
								x, y, genRGBString(tc.cost, optRun.minCost, optRun.maxCost),
								foundChosen, highlightColor));

					std::ostringstream lineTwo;
					std::ostringstream lineThree;
					lineTwo << std::fixed << std::setprecision(3) << tc.cost;
					lineThree << std::fixed << std::setprecision(3)
						<< tc.er.falsePositives << "/" << tc.er.averageRecognitionRuntime;

					textNodes.push_back(genTextSVG(std::to_string(tc.index),
								lineTwo.str(), lineThree.str(),
								x, y));

					index++;
				}
			}

			ptree root;
			ptree svg;

			svg.put("<xmlattr>.xmlns", "http://www.w3.org/2000/svg") ;
			svg.put("<xmlattr>.width", std::to_string(imgWidth)) ;
			svg.put("<xmlattr>.height", std::to_string(imgHeight)) ;
			svg.put("<xmlattr>.background", "white") ;

			svg.add_child("rect", genRectSVG(0, 0, imgWidth, imgHeight, "white"));
			svg.add_child("polyline", genPolylineSVG(linePoints));

			for (unsigned int j = 0; j < circleNodes.size(); ++j)
			{
				svg.add_child("circle", circleNodes[j]);
			}

			for (unsigned int j = 0; j < textNodes.size(); ++j)
			{
				svg.add_child("text", textNodes[j]);
			}

			for (unsigned int j = 0; j < seperatorLinePositions[i].size(); ++j)
			{
				svg.add_child("line", genSeparatorLineSVG(imgHeight - seperatorLinePositions[i][j],
							PADDING, imgWidth - PADDING));
			}

			root.add_child("svg", svg);
			roots.push_back(root);
		}

		return roots;
	}

	ptree SVGHelper::genCircleSVG(double radius, int x, int y, const std::string& circleColor,
			bool highlight, const std::string& highlightColor)
	{
		ptree circleNode;
		circleNode.put("<xmlattr>.cx", std::to_string(x));
		circleNode.put("<xmlattr>.cy", std::to_string(y));
		circleNode.put("<xmlattr>.r",  std::to_string(radius));
		circleNode.put("<xmlattr>.fill", circleColor);
		if (highlight)
		{
			circleNode.put("<xmlattr>.stroke", highlightColor);
			circleNode.put("<xmlattr>.stroke-width", std::to_string(HIGHLIGHT_WIDTH));
		}
		return circleNode;
	}

	ptree SVGHelper::genTextSVG(const std::string& lineOne, const std::string& lineTwo,
			const std::string& lineThree, int x, int y)
	{
		ptree textNode;
		textNode.put("<xmlattr>.y", std::to_string(y));
		textNode.put("<xmlattr>.x", std::to_string(x));
		textNode.put("<xmlattr>.fill", "black");
		textNode.put("<xmlattr>.font-family", "Courier New");
		textNode.put("<xmlattr>.text-anchor", "middle");
		textNode.put("<xmlattr>.font-weight", "bold");
		textNode.put("<xmlattr>.font-size", std::to_string(FONT_SIZE) + "px") ;

		ptree lOne;
		lOne.put("", lineOne);
		lOne.put("<xmlattr>.x", std::to_string(x + FONT_SIZE / 4));
		lOne.put("<xmlattr>.dy", - (int) FONT_SIZE * 1.25);
		lOne.put("<xmlattr>.font-style", "italic");

		ptree lTwo;
		lTwo.put("", lineTwo);
		lTwo.put("<xmlattr>.x", std::to_string(x + FONT_SIZE / 4));
		lTwo.put("<xmlattr>.dy", FONT_SIZE * 1.25);

		ptree lThree;
		lThree.put("", lineThree);
		lThree.put("<xmlattr>.x", std::to_string(x));
		lThree.put("<xmlattr>.dy", FONT_SIZE * 1.25);

		textNode.add_child("tspan", lOne);
		textNode.add_child("tspan", lTwo);
		textNode.add_child("tspan", lThree);

		return textNode;
	}

	ptree SVGHelper::genSeparatorLineSVG(int y, int x1, int x2)
	{
		ptree sepeartorLineNode;
		sepeartorLineNode.put("<xmlattr>.stroke-dasharray", "5, 5");
		sepeartorLineNode.put("<xmlattr>.y1", std::to_string(y));
		sepeartorLineNode.put("<xmlattr>.y2", std::to_string(y));
		sepeartorLineNode.put("<xmlattr>.x1", std::to_string(x1));
		sepeartorLineNode.put("<xmlattr>.x2", std::to_string(x2));
		sepeartorLineNode.put("<xmlattr>.stroke", "black");
		sepeartorLineNode.put("<xmlattr>.stroke-width", "3");

		return sepeartorLineNode;
	}

	ptree SVGHelper::genPolylineSVG(std::vector<std::string> linePoints)
	{
		std::string linePointsString = "";
		if (linePoints.size() != 0)
		{
			linePointsString += linePoints[0];
			for (unsigned int i = 1; i < linePoints.size(); ++i)
			{
				linePointsString += " " + linePoints[i];
			}
		}

		ptree lineNode;
		lineNode.put("<xmlattr>.points", linePointsString);
		lineNode.put("<xmlattr>.style", "fill:none;stroke:"
				+ COLOR_HIGHLIGHT + ";stroke-linejoin:round;stroke-width:"
				+ std::to_string(LINE_WIDTH));

		return lineNode;
	}

	ptree SVGHelper::genRectSVG(int x, int y, unsigned int width,
			unsigned int height, const std::string& colorString)
	{
		ptree rectNode;
		rectNode.put("<xmlattr>.x", std::to_string(x));
		rectNode.put("<xmlattr>.y", std::to_string(y));
		rectNode.put("<xmlattr>.width", std::to_string(width));
		rectNode.put("<xmlattr>.height", std::to_string(height));
		rectNode.put("<xmlattr>.style", "fill:" + colorString);

		return rectNode;
	}


	ptree SVGHelper::createXML(const std::string& patternName)
	{
		ptree root;
		ptree optimizationRun;
		optimizationRun.put("<xmlattr>." + ATR_PATTERN_NAME, patternName) ;
		for (unsigned int i = 0; i < mAllEvaluationRounds.size(); ++i)
		{
			OptimizationRound optRound = mAllEvaluationRounds[i];
			ptree erNode = genEvaluationRoundXML(i, optRound.selectedTopologyIndex);

			for (unsigned int j = 0; j < optRound.evaluatedTopologies.size(); ++j)
			{
				ptree tcNode = genTopologyContainerXML(optRound.evaluatedTopologies[j]);
				erNode.add_child(NAME_TOPOLOGY, tcNode);
			}
			optimizationRun.add_child(NAME_ROUND, erNode);
		}
		root.add_child(NAME_OPTIMIZATION_RUN, optimizationRun);
		return root;
	}

	ptree SVGHelper::genTopologyContainerXML(TopologyContainer tc)
	{
		ptree tcNode;
		tcNode.put("<xmlattr>." + ATR_EVALUATION_RESULT, std::to_string(tc.cost)) ;
		tcNode.put("<xmlattr>." + ATR_RELATION_IDS, tc.identifier);
		return tcNode;
	}

	ptree SVGHelper::genEvaluationRoundXML(unsigned int roundNumber, unsigned int chosenIndex)
	{
		ptree erNode;
		erNode.put("<xmlattr>." + ATR_ROUND_NUMBER, std::to_string(roundNumber)) ;
		erNode.put("<xmlattr>." + ATR_CHOSEN_INDEX, std::to_string(chosenIndex)) ;
		return erNode;
	}


	std::string SVGHelper::genRGBString(double cost, double minCost, double maxCost)
	{
		if (cost == std::numeric_limits<double>::infinity())
			return "rgb(255, 0, 0)";

		double diff = maxCost - minCost;
		double halfWay = diff / 2;
		double ratio = 255 / halfWay;
		double normedCost= cost - minCost;
		double redValue, greenValue, blueValue = 0;
		if (normedCost < halfWay)
		{
			redValue = normedCost * ratio;
			greenValue = 255;
		} 
		else 
		{
			redValue = 255;
			greenValue = (2 * halfWay - normedCost) * ratio;
		}

		return "rgb(" + std::to_string((int) redValue) + ","
			+ std::to_string((int) greenValue) + ","
			+ std::to_string((int) blueValue) + ")";
	}

	double SVGHelper::calculateCircleRadius(double averageRecognitionRuntime,
			double minAverageRecognitionRuntime,
			double maxAverageRecognitionRuntime)
	{
		double diff = maxAverageRecognitionRuntime - minAverageRecognitionRuntime;
		double ratio = (averageRecognitionRuntime - minAverageRecognitionRuntime) / diff;
		return ratio * (MAX_CIRCLE_RADIUS - MIN_CIRCLE_RADIUS) + MIN_CIRCLE_RADIUS;
	}
}
