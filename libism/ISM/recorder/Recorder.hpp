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

#include "common_type/ObjectSet.hpp"
#include "common_type/Object.hpp"
#include <string>
#include <boost/shared_ptr.hpp>
#include "utility/TableHelper.hpp"

namespace ISM {

  /**
   * Recorder class. Wrapper for TableHelper that simplifies adding object configurations to already recorded training data. 
   */
  class Recorder {
    TableHelperPtr tableHelper;
  public:

    /**
     * Create training data recording interface to an sqlite db.
     *
     * @param dbfilename Db into which training data is to be stored.
     */
    Recorder(const std::string& dbfilename/* = "record.sqlite"*/);

    //~Recorder();

    /**
     * Add object configuration of a certain scene to already recorded training data.
     *
     * @param set One example of a spatial configuration of objects.
     * @param patternName Scene to which object configuration belongs.
     */
    void insert(const ObjectSetPtr& set, const std::string& patternName);
  };
  typedef boost::shared_ptr<Recorder> RecorderPtr;
}
