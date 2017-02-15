/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "DataMerger.hpp"
#include "../utility/TableHelper.hpp"

#include <set>

namespace ISM
{
void DataMerger::merge(const std::string& targetFile, const std::vector<std::string>& sourceFiles, bool mergeRecordings, bool mergeModels)
{
    TableHelper target(targetFile);
    for (const std::string& sourceFile : sourceFiles)
    {
        TableHelper source(sourceFile);
        std::cout << sourceFile << ":" << std::endl;

        if (mergeRecordings)
        {
            std::cout << "merging recordings" << std::endl;
            std::vector<std::string> patternNames = source.getRecordedPatternNames();
            std::cout << "found " << patternNames.size() << " patterns" << std::endl;
            for (const std::string& patternName : patternNames)
            {
                std::cout << "merge pattern " << patternName << std::endl;
                RecordedPatternPtr pattern = source.getRecordedPattern(patternName);
                for (const ObjectSetPtr& set : pattern->objectSets)
                {
                    target.insertRecordedObjectSet(set, patternName);
                    std::cout << ".";
                    std::cout.flush();
                }
                std::cout << "done" << std::endl;
            }
        }

        if (mergeModels)
        {
            std::cout << "merging models" << std::endl;
            std::set<std::string> objectTypes = source.getObjectTypes();
            std::cout << "found " << objectTypes.size() << " object types" << std::endl;
            ObjectTypeToVoteMap typeToVoteMap = source.getVoteSpecifiersForObjectTypes(objectTypes);
            std::set<std::string> patternNames;
            for (const std::pair<std::string, std::vector<VoteSpecifierPtr>>& typeToVoteVecPair : typeToVoteMap)
            {
                std::string type = typeToVoteVecPair.first;
                std::vector<VoteSpecifierPtr> voteVec = typeToVoteVecPair.second;
                std::cout << "found " << voteVec.size() << " votes for object type " << type << std::endl;
                for (VoteSpecifierPtr& vote : voteVec) {
                    patternNames.insert(vote->patternName);
                    target.insertModelVoteSpecifier(vote);
                    std::cout << ".";
                    std::cout.flush();
                }
                std::cout << "done" << std::endl;
            }

            PatternNameToPatternMap patternMap = source.getPatternDefinitionsByName(patternNames);
            std::cout << "found " << patternMap.size() << " patterns" << std::endl;
            for (const std::pair<std::string, PatternPtr>& patternNameToPattern : patternMap)
            {
                PatternPtr pattern = patternNameToPattern.second;
                target.upsertModelPattern(pattern->name, pattern->expectedMaxWeight);
                std::cout << ".";
                std::cout.flush();
            }
            std::cout << "done" << std::endl;
        }
    }
}
}
