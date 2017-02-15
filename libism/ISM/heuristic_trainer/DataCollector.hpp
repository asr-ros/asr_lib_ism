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

#include <boost/shared_ptr.hpp>
#include "common_type/Tracks.hpp"
#include "common_type/Track.hpp"
#include <vector>

namespace ISM {
    struct TracksWithRef {
            TracksPtr tracks;
            TrackPtr refTrack;
            TracksWithRef(TracksPtr tracks, TrackPtr refTrack) : tracks(tracks), refTrack(refTrack) {};
    };
    typedef boost::shared_ptr<TracksWithRef> TracksWithRefPtr;

    struct CollectedData {
            bool shouldCollect;
            std::vector<TracksWithRef> tracksWithRef;
    };

    typedef boost::shared_ptr<CollectedData> CollectedDataPtr;

    class DataCollector {
        private:
            DataCollector() {};
            ~DataCollector() {};
        public:
            static CollectedDataPtr getData() {
                static CollectedDataPtr data;
                if (!data) {
                    data = CollectedDataPtr(new CollectedData());
                    data->shouldCollect = false;
                }
                return data;
            }

            static void setCollect(bool newCollect) {
                getData()->shouldCollect = newCollect;
            }

            static bool shouldCollect() {
                return getData()->shouldCollect;
            }

            static void release() {
                bool shouldCollect = getData()->shouldCollect;
                getData().reset();
                getData()->shouldCollect = shouldCollect;
            }
    };
}
