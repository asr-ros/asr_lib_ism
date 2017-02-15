/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "ManuallyDefPseudoHeuristic.hpp"
#include <boost/lexical_cast.hpp>
namespace ISM
{

  ManuallyDefPseudoHeuristic::ManuallyDefPseudoHeuristic(std::vector<std::pair<std::vector<ClusterObject>, uint16_t>>
		  clusters) : Heuristic("ManuallyDefPseudoHeuristic"), mClusters(clusters)
  {

  }
  ManuallyDefPseudoHeuristic::~ManuallyDefPseudoHeuristic()
  {
  }
  void ManuallyDefPseudoHeuristic::applyHeuristic(const TracksPtr& tracks)
  {
	  std::vector<TrackPtr> result;
      for(std::pair<std::vector<ClusterObject>, uint16_t>& clusterObjectsToCluster : mClusters)
      {
    	  for(ClusterObject& clusterObject : clusterObjectsToCluster.first)
    	  {
    		  bool found = false;
              for(TrackPtr& track : tracks->tracks)
              {
            	  if(clusterObject.isCluster)
            	  {
            		  if(track->type.find("_sub") != std::string::npos)
            		  {
            			  std::string temp = track->type;
            			  temp.erase(temp.begin(), temp.begin() + temp.find_first_of("_sub") + 4);
            			  if(boost::get<uint16_t>(clusterObject.mObjectOrCluster) ==
            					  boost::lexical_cast<uint16_t>(temp))
            			  {
            				  found = true;
            				  result.push_back(track);
            			  }
            		  }
            	  } else if(track->type == boost::get<std::string>(clusterObject.mObjectOrCluster))
            	  {
            		  found = true;
            		  result.push_back(track);
            	  }
              }
              if(!found)
              {
            	  result.clear();
            	  break;
              }
    	  }
    	  if(result.size() == clusterObjectsToCluster.first.size())
    	  {
              cluster = TracksPtr(new Tracks(result));
    		  confidence = 1.0;
    		  break;
    	  }
      }
  }
}



