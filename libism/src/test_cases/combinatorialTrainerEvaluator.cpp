/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <boost/filesystem/path.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ISM/combinatorial_trainer/CombinatorialTrainer.hpp>
#include <ISM/combinatorial_trainer/CombinatorialTrainerParameters.hpp>
#include <ISM/utility/TableHelper.hpp>
#include "randomDemoRecorder.hpp"

#include <sys/time.h>
#include <string>
#include <chrono>
#include <vector>
#include <string>

#include <random>
#include <algorithm>

using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using boost::filesystem::path;

	std::vector<ISM::CombinatorialTrainerParameters> parameters;

	std::vector<unsigned> objectCounts = {
		5, 10, 15
	};

	std::vector<unsigned> posesPerObjectCount = {
		10, 25, 50, 100, 200, 400
	};

	ISM::CombinatorialTrainerParameters getDefaultParams()
	{
		ISM::CombinatorialTrainerParameters params;

		params.evaluator.evaluatorId = 0;
		params.evaluator.countThreadsTester = 8;
		params.evaluator.confidenceThreshold = 0.99;
		params.topologyGenerator.topologyGeneratorId = 0;
		params.topologyGenerator.swapRelations = true;
		params.topologyGenerator.maxNeighbourCount = 30;

		params.treeValidator.treeValidatorId = 0;
		params.treeValidator.maxTreeHeight = 3;

		params.optimizationStrategy.maxNeighbourCount = 30;
		params.optimizationStrategy.randomWalkProbability = 0.0;
		params.optimizationStrategy.randomRestartProbability = 0.0;

		params.evaluator.binSize = 0.1;

		params.costFunction.alpha = 5;
		params.costFunction.beta = 1;

		params.voxelGrid.useVoxelGridFilter = false;
		params.voxelGrid.posGridSize = 0.1;
		params.voxelGrid.angleGridSize = 0.52;
		params.voxelGrid.posErrTolerance = 0.1;
		params.voxelGrid.recognitionBinSize = 0.1;
		params.voxelGrid.tolearnaceDupMethod = ISM::PosOrientVoxelGrid::HandleErrorDuplicates::LEAVE_UNCHANGED;
		params.voxelGrid.tolDupNumThreshold = 0;

		params.general.storeFullyMeshedISM = false;
		params.general.storeStartTopologyISM = false;

		return params;
	}

	void writeFile(const std::string & directoryPath, const std::string & filenName, std::ostringstream & content)
	{
		std::string filePath = directoryPath + "/" + filenName;
		std::ofstream file;
		std::ios_base::iostate exceptionMask = file.exceptions() | std::ios::failbit | std::ios::badbit;
		file.exceptions(exceptionMask);
		try
		{
			file.open(filePath);
			file << content.str();
			file.flush();
			file.close();
		}
		catch (std::ios_base::failure& e)
		{
			std::cerr << e.what() << "\n";
		}
	}

	void runOptimization(ISM::CombinatorialTrainerParameters params, std::ostringstream & os)
	{
		ptime t1(boost::posix_time::microsec_clock::local_time());

		ISM::CombinatorialTrainerPtr combinatorialTrainer =
			ISM::CombinatorialTrainerPtr(new ISM::CombinatorialTrainer(params));
		combinatorialTrainer->learn();

		ptime t2(boost::posix_time::microsec_clock::local_time());
		time_duration td = t2 - t1;

		boost::filesystem::path path = params.general.dbfilename;

		long secs = td.total_seconds() % 60;
		long mins = std::floor(td.total_seconds() / 60.);

		os << ", " << mins << "." << secs;
	}

	void prepareSceneTests(const std::string & outputPath)
	{
		parameters.clear();

		ISM::CombinatorialTrainerParameters params = getDefaultParams();

		//Uncomment and specify path to the sqlite database that contains the scene that should be trained.
		//params.general.dbfilename = "path_to_scene";

		params.general.outputDataPath = outputPath;
		parameters.push_back(params);
	}

	void initTestSets(const std::string & testSetFolderPath)
	{
		if(!boost::filesystem::exists(testSetFolderPath))
			boost::filesystem::create_directories(testSetFolderPath);

		for (unsigned int i = 0; i < parameters.size(); ++i) {
			ISM::CombinatorialTrainerParameters params = parameters[i];

			path dbPath(params.general.dbfilename);

			path validTestSetsPath(testSetFolderPath + "/validTestSets_" + dbPath.stem().string() + ".sqlite");
			params.general.storeValidTestSetsTo = validTestSetsPath.string();

			path invalidTestSetsPath(testSetFolderPath + "/invalidTestSets_" + dbPath.stem().string() + ".sqlite");
			params.general.storeInvalidTestSetsTo = invalidTestSetsPath.string();

			ISM::CombinatorialTrainerPtr combinatorialTrainer =
				ISM::CombinatorialTrainerPtr(new ISM::CombinatorialTrainer(params));

			parameters[i].general.loadValidTestSetsFrom = params.general.storeValidTestSetsTo;
			parameters[i].general.loadInvalidTestSetsFrom = params.general.storeInvalidTestSetsTo;

			parameters[i].general.storeValidTestSetsTo = "";
			parameters[i].general.storeInvalidTestSetsTo = "";
		}
	}

	void runSceneTests(std::ostringstream & os)
	{
		for (unsigned int i = 0; i < parameters.size(); ++i) {
			ISM::CombinatorialTrainerParameters params = parameters[i];

			path p = path(params.general.dbfilename);
			os << p.stem().string();

			//HillClimbing (strict)
			params.optimizationAlgorithm.optimizationAlgorithmId = 0;
			runOptimization(params, os);

			//SimulatedAnnealing
			params.optimizationAlgorithm.optimizationAlgorithmId = 2;
			params.optimizationAlgorithm.startTemperature = 10;
			params.optimizationAlgorithm.endTemperature = 0.00001;
			params.optimizationAlgorithm.temperatureFactor = 0.9;
			runOptimization(params, os);

			//RecordHunt
			params.optimizationAlgorithm.optimizationAlgorithmId = 3;
			params.optimizationAlgorithm.initialAcceptableCostDelta = 0.05;
			params.optimizationAlgorithm.costDeltaDecreaseFactor = 0.01;
			runOptimization(params, os);

			os << std::endl;
		}
	}

	void testPerformance(const std::string & outputPath)
	{
		ISM::CombinatorialTrainerParameters params = getDefaultParams();

		std::string demoRecordingPath = outputPath + "/demoRecording.sqlite";

		params.general.dbfilename = demoRecordingPath;
		params.general.outputDataPath = outputPath;

		RandomDemoRecorder randomDemoRecorder = *(new RandomDemoRecorder);

		ISM::TableHelperPtr tableHelper =
			ISM::TableHelperPtr(new ISM::TableHelper(demoRecordingPath));

		std::ostringstream os;

		for (unsigned int i = 0; i < objectCounts.size(); ++i)
		{
			os << "Scene, Hill Climbing, Simulated Annealing, Record Hunt" << std::endl;

			for (unsigned int j = 0; j < posesPerObjectCount.size(); ++j)
			{
				tableHelper->dropRecordTables();
				randomDemoRecorder.generateDemoRecording(demoRecordingPath, objectCounts[i],
						posesPerObjectCount[j], false);

				os << objectCounts[i] << "-" << posesPerObjectCount[j];

				//HillClimbing (strict)
				params.optimizationAlgorithm.optimizationAlgorithmId = 0;
				runOptimization(params, os);

				//SimulatedAnnealing
				params.optimizationAlgorithm.optimizationAlgorithmId = 2;
				params.optimizationAlgorithm.startTemperature = 10;
				params.optimizationAlgorithm.endTemperature = 0.00001;
				params.optimizationAlgorithm.temperatureFactor = 0.9;
				runOptimization(params, os);

				//RecordHunt
				params.optimizationAlgorithm.optimizationAlgorithmId = 3;
				params.optimizationAlgorithm.initialAcceptableCostDelta = 0.05;
				params.optimizationAlgorithm.costDeltaDecreaseFactor = 0.01;
				runOptimization(params, os);

				os << std::endl;
			}

			os << std::endl;
		}

		writeFile(outputPath, "runtimes.csv", os);
	}

	void testScenes(const std::string & outputPath)
	{
		std::ostringstream os;
		os << "Scene, Hill Climbing, Simulated Annealing, Record Hunt" << std::endl;

		prepareSceneTests(outputPath);
		initTestSets(outputPath + "/TestSets");
		runSceneTests(os);

		writeFile(outputPath, "runtimes.csv", os);
	}

	int main()
	{
		//Uncomment and and path to the folder where the ouptu should be stored.

		//std::string outputPath = "output_path";
		std::string performanceTestPath = outputPath + "/performanceTest";
		std::string sceneTestPath = outputPath + "/sceneTest";

		if(!boost::filesystem::exists(outputPath))
			boost::filesystem::create_directories(outputPath);

		if(!boost::filesystem::exists(performanceTestPath))
			boost::filesystem::create_directories(performanceTestPath);

		if(!boost::filesystem::exists(sceneTestPath))
			boost::filesystem::create_directories(sceneTestPath);

		//Test randomly generated scenes specified by the values in objectCounts and posesPerObjectCount.
		testPerformance(performanceTestPath);

		//Test scenes specified in prepareSceneTests
		testScenes(sceneTestPath);
	}

