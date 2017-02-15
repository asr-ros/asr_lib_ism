/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "randomDemoRecorder.hpp"

using namespace ISM;
using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;


double RandomDemoRecorder::frand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void RandomDemoRecorder::generateDemoRecording (string dbfile, unsigned objects, unsigned timesteps, bool genRanOrientsAdditionally)
{
  //string dbfile = "/homes/students/hanselma/testDBs/demo_recordings/record.sqlite.demo_recording_" + to_string(objects) + "_" + to_string(timesteps);
  double directioncos = cos(0.0 * (boost::math::constants::pi<double>() / 180.0));
  double directionsin = sin(0.0 * (boost::math::constants::pi<double>() / 180.0));
  RecorderPtr rec (new Recorder(dbfile));
  //We can leave y constant to simulate a plane
  double y = 0;
  vector<TrackPtr> trackVector;
  for (unsigned object = 0; object < objects; ++object)
    {
      TrackPtr track(new Track());
      double x = frand(0, 1.0);
      double z = frand(0, 1.0);    
      double angle = frand(0, boost::math::constants::pi<double>() * 2);
      for (unsigned timestep = 0; timestep < timesteps; ++timestep)
	{
	  //Calculates direction based on current direction. Sharp angles are possible, but only in edge cases
	  angle += (frand(boost::math::constants::pi<double>() * (-1), boost::math::constants::pi<double>())) * pow(frand(0, 1.0), 2); 
	  double u = frand(0, 1.0);
	  double radius = frand(0, 0.01) * sqrt(u);
	  double xTranslation = radius * cos(angle);
	  double zTranslation = radius * sin(angle);
	  x += xTranslation;
	  if (x > 1)
	    {
	      x -= 1;
	    }
	  else if (x < -1)
	    {
	      x += 1;
	    }
	  z += zTranslation;
	  if (z > 1)
	    {
	      z -= 1;
	    }
	  else if (z < -1)
	    {
	      z += 1;
	    }

      ObjectPtr o;
      if(genRanOrientsAdditionally)
      {
        o = ObjectPtr(new Object(string("testobj") + to_string(object), new Pose(new Point(x, y, z), new Quaternion(0.0, sin(angle), 0.0, cos(angle))), to_string(object)));
      }
      else
      {
        o = ObjectPtr(new Object(string("testobj") + to_string(object), new Pose(new Point(x, y, z), new Quaternion(directionsin, 0.0, 0.0, directioncos)), to_string(object)));
      }
      track->objects.push_back(o);
	}
      trackVector.push_back(track);	
    }
  TracksPtr tracks(new Tracks(trackVector));
  vector<ObjectSetPtr> objectSets = tracks->toObjectSetVector();
  for (ObjectSetPtr objectSet : objectSets)
    {
      rec->insert(objectSet, "demo_recording_" + to_string(objects) + "_" + to_string(timesteps));
    }
  /*  std::vector<std::string> modeldatabases = 
    {
      "/homes/students/hanselma/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objects) + "_" + to_string(timesteps) + "_best",
      "/homes/students/hanselma/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objects) + "_" + to_string(timesteps) + "_best_naive",
      "/homes/students/hanselma/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objects) + "_" + to_string(timesteps) + "_star",
      "/homes/students/hanselma/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objects) + "_" + to_string(timesteps) + "_fullyMeshed",
      "/homes/students/hanselma/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objects) + "_" + to_string(timesteps) + "_fullyMeshed_naive" 
    };
  std::cout<<"creating model dbs\n";
  for (string model : modeldatabases)
    {
      cout<<"current path is "<<model<<endl;
      for (ObjectSetPtr objectSet : objectSets)
	{
	  TableHelperPtr localTableHelper(new TableHelper(model));
	  localTableHelper->dropRecordTables();
	  RecorderPtr localrec(new Recorder(model));
	  localrec->insert(objectSet, "demo_recording_" + to_string(objects) + "_" + to_string(timesteps));
	}
	}*/
  printYellow("Done. Objects are now stored in " + dbfile + "\n");
}

