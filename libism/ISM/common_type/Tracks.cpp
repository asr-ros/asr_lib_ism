/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Tracks.hpp"

namespace ISM {
    Tracks::Tracks(std::vector<ObjectSetPtr> sets) {
        size_t setIdx = 0;
        for (ObjectSetPtr& set : sets) {
      for (ObjectPtr& object : set->objects) {
                TrackPtr track = getTrackByTypeAndId(object->type, object->observedId);
                if (!track) {
                    track = TrackPtr(new Track(object->type, object->observedId));
                    tracks.push_back(track);

                    // fill up with empty object pointers
                    for (size_t i = 0; i < setIdx; i++) {
                        track->objects.push_back(ObjectPtr());
                    }
                }

                track->objects.push_back(object);
            }
            // fill up tracks that had no object in this frame
            for (TrackPtr& track : tracks) {
                for (size_t i = track->objects.size(); i <= setIdx; i++) {
                    track->objects.push_back(ObjectPtr());
                }
            }
            setIdx++;
        }

        for (TrackPtr& track : tracks) {
            track->calculateWeight();
        }
    }

    Tracks::Tracks(std::vector<TrackPtr> tracks) {
        this->tracks = tracks;
    }

    TrackPtr Tracks::getTrackByTypeAndId(std::string type, std::string observedId) {
        for (TrackPtr& track : tracks) {
            if (track->type == type && track->observedId == observedId) {
                return track;
            }
        }

        return TrackPtr();
    }

    std::vector<ObjectSetPtr> Tracks::toObjectSetVector() {
        std::vector<ObjectSetPtr> ret;
        if (tracks.size() > 0) {
            for (size_t i = 0; i < tracks[0]->objects.size(); i++) {
                ObjectSetPtr set(new ObjectSet());
                for (TrackPtr& track : tracks) {
                    if (track->objects[i]) {
                        set->insert(track->objects[i]);
                    }
                }
                ret.push_back(set);
            }
        }

        return ret;
    }

    void Tracks::replace(std::vector<TrackPtr> eraseTracks, TrackPtr newTrack) {
        for (TrackPtr& eraseTrack : eraseTracks) {
            for (size_t i = 0; i < tracks.size(); i++) {
                TrackPtr track = tracks[i];
                if (eraseTrack == track) {
                    tracks.erase(tracks.begin() + i);
                    break;
                }
            }
        }

        tracks.push_back(newTrack);
    }
}
