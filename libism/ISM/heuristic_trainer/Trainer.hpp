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

#include <string>
#include <sstream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "common_type/ObjectSet.hpp"
#include "common_type/Object.hpp"
#include "utility/TableHelper.hpp"
#include "common_type/Pose.hpp"
#include "Heuristic.hpp"
#include "ManuallyDefPseudoHeuristic.hpp"

namespace ISM {

  /**
   * Trainer class. Learns scene models from training data. See Meissner et al. 2013 in Section IV and V (A).
   */
    class Trainer {
        TableHelperPtr tableHelper;
        RecordedPatternPtr recordedPattern;
        PointPtr absoluteReferencePoint;
        int skips;
        bool useClustering;
        bool mUseManualDefHeuristic;
        bool mUsePredefinedRefs;
        std::vector<std::pair<std::vector<ManuallyDefPseudoHeuristic::ClusterObject>, uint16_t>>
            mClusterForManualDefHeuristic;
        std::map<std::string, std::string> mPatternToTypesOfPredefinedRefs;

        public:

      //Some parameters for heuristics used here
      double staticBreakRatio, togetherRatio, maxAngleDeviation;

      void setClusterForManualDefHeuristic(std::vector<std::pair<std::vector<ManuallyDefPseudoHeuristic::ClusterObject>,
    		  uint16_t>>);
      void setPredefinedRefs(std::map<std::string, std::string>& refs);

      /**
       * Create training interface to an sqlite db.
       *
       * @param dbfilename Db from which training data is taken and into which scene models are written.
       */
            Trainer(std::string dbfilename = "record.sqlite", bool dropOldModelTables = false);

      /**
       * Perform scene model learning on data from sqlite db loaded beforehand.
       */
            void trainPattern();
            void trainPattern(const std::string& patternName);
            void setSkipsPerCycle(const int skips);

      /**
       * Whether to learn one ism on training data or rather a tree of isms.
       *
       * @param useClustering Decides whether to subdivide scene elements into clusters based on heuristics or the leave them all in one set before learning of isms.  
       */
            void setUseClustering(const bool useClustering);

        private:
            void learn();
            HeuristicPtr findHeuristicMatch(const TracksPtr& tracks);
            TrackPtr doTraining(const std::vector<ObjectSetPtr> sets, std::string patternName);
    };
  typedef boost::shared_ptr<Trainer> TrainerPtr;
}
