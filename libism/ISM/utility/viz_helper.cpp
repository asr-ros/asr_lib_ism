/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "viz_helper.hpp"
#include <string>

namespace ISM
{

    /* only working because the shape-based recognizer sets the observedId with the object color */
    std::vector<float> getColorOfObject(const ISM::Object object){
        std::string observedId = object.observedId;
        std::vector<float> rgba;
        if ( ( observedId.length() == 12 ) && ( observedId.find_first_not_of("0123456789") == std::string::npos ) )
        {
            try
            {
                for (int i = 0; i <= 3; i++)
                {
                    std::string temp;

                    temp = observedId.substr( (i * 3), 3 );
                    rgba.push_back(std::stof(temp) / 100.0);
                }
            }
            catch (std::invalid_argument& ia)
            {
                rgba.clear();
                rgba.push_back(0);
                rgba.push_back(0);
                rgba.push_back(0);
                rgba.push_back(1);
            }
        }else{
            rgba.push_back(0);
            rgba.push_back(0);
            rgba.push_back(0);
            rgba.push_back(0);
        }
        return rgba;
    }

    std::vector<float> getColorOfObject(const ISM::ObjectPtr object_ptr){
      return getColorOfObject(*object_ptr);
    }
}
