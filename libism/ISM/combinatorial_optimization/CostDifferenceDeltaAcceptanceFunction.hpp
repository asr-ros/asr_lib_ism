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
#include <random>

#include "CoolingSchedule.hpp"

namespace ISM {

class ExponentialCoolingSchedule : public CoolingSchedule
{
	public:
		ExponentialCoolingSchedule(double startTemperature, double endTemperature, unsigned repetitionsBeforeUpdate,
				double temperatureFactor)
			: CoolingSchedule(startTemperature, endTemperature, repetitionsBeforeUpdate)
			, mTemperatureFactor(temperatureFactor)
		{}

		bool isNewCostAcceptable(double newCost, double oldCost);
		void update();

	private:
		double mTemperatureFactor;

		std::default_random_engine mGenerator;
		std::uniform_real_distribution<double> mDistribution = std::uniform_real_distribution<double>(0.0, 1.0);

}; typedef boost::shared_ptr<ExponentialCoolingSchedule> ExponentialCoolingSchedulePtr;

}

