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
#include <string>
#include <map>
#include <set>

#include "soci/src/core/soci.h"
#include "soci/src/backends/sqlite3/soci-sqlite3.h"
#include "common_type/Object.hpp"
#include "common_type/ObjectSet.hpp"
#include "common_type/RecordedPattern.hpp"
#include "common_type/VoteSpecifier.hpp"
#include "common_type/Pattern.hpp"

using namespace soci;
namespace ISM {
  typedef std::map<std::string, std::vector<VoteSpecifierPtr> > ObjectTypeToVoteMap;
  typedef std::map<std::string, PatternPtr> PatternNameToPatternMap;
  //Access votes for a specific object (given as combination of object type and id).
  typedef std::map<std::string, std::map<std::string, std::vector<VoteSpecifierPtr> > > ObjectToVoteMap;
  //Access votes for a specific object (given as combination of object type and id) in a specific scene.
  typedef std::map<std::string, std::map<std::string, std::map<std::string, std::vector<VoteSpecifierPtr> > > > PatternToObjectToVoteMap;

  /**
   * TableHelper class. Interface to training data and scene models (resp. votes) sqlite db.
   */
  class TableHelper {
    boost::shared_ptr<session> sqlite;
  public:
    /* util */
    TableHelper(std::string dbfilename = "record.sqlite");

    ~TableHelper();

    void createTablesIfNecessary() const;
    void createColumnsIfNecessary();
    void createTable(const std::string& tablename, const std::string& sql) const;
    void dropTable(const std::string& tablename) const;
    void dropRecordTables() const;
    void dropTables() const;
    void dropModelTables() const;

    int getLastInsertId(const std::string& tablename) const;

    /* Functions that are used by recorder to insert object data into table. */
    int insertRecordedObject(const boost::shared_ptr<Object>& o, int setId) const;
    int insertRecordedObjectSet(const boost::shared_ptr<ObjectSet>& os, const std::string& patternName) const;
    int insertRecordedPattern(const std::string& patternName) const;
    int insertRecordedPattern(const RecordedPatternPtr& pattern) const;

    /**
     * Get numeric id that relates training data with the scene it represents.
     *
     * @param patternName Name of scene represented as string.
     * @return Id that associates scene name to object sets resp. configurations.
     */
    int getRecordedPatternId(const std::string& patternName) const;
    int ensureRecordedPatternName(const std::string& patternName) const;
      
    /**
     * Extracts all names of scenes for which training data has been recorded and stored in the given db. 
     *
     * @return Scene names given to recordings stored here.
     */
    std::vector<std::string> getRecordedPatternNames() const;

    const RecordedPatternPtr getRecordedPattern(const std::string& patternName) const;
    const std::vector<int> getSetIds() const;
    const ObjectSetPtr getRecordedObjectSet(int setId) const;

    /* model */
    int insertModelVoteSpecifier(const VoteSpecifierPtr& vote) const;
    int insertModelPattern(const std::string& patternName) const;
    int upsertModelPattern(const std::string& patternName, int expectedMaxWeight) const;
    int insertModelObjectType(const std::string& objectType) const;

    /**
     * Get numeric id that relates (sub-)scene name to the ism (in the tree) it represents.
     *
     * @param patternName Name of reference node of ism in the tree.
     * @return Id that associates reference name to object poses relative to this reference.
     */
    int getModelPatternId(const std::string& patternName) const;
    int getModelObjectTypeId(const std::string& objectType) const;
    int ensureModelPatternName(const std::string& patternName) const;
    int ensureModelObjectType(const std::string& objectType) const;

    /**
     * Extracts all string identifiers of the reference nodes in the ism trees learnt from training data in this db.
     *
     * @return Names of scenes and subscenes in the ism tree.
     */
    std::vector<std::string> getModelPatternNames() const;

    const PatternNameToPatternMap getPatternDefinitionsByName(const std::set<std::string>& patternNames) const;

    /**
     * Get types of all objects in the ism trees learnt from training data in this db.
     *
     * @return Different types of objects encountered as scene elements (reference and non-reference nodes).
     */
    const std::set<std::string> getObjectTypes() const;

    /**
     * Get ressource paths of all objects in record.
     *
     * @return Map where type is mapped to ressource path.
     */
    const std::map<std::string, boost::filesystem::path> getRessourcePaths() const;

    const std::map<std::string, std::map<std::string, std::string>> getModelWeightsPerTypeAndId() const;

    /**
     * Get all objects in the ism trees learnt from training data in this db.
     *
     * @return Objects in ism tree represented as combination of object type and identifier (the latter differentiating instances of the same type).
     */
    const std::set<std::pair<std::string, std::string> > getObjectTypesAndIdsFromModelObjects() const;
    const std::set<std::pair<std::string, std::string> > getObjectTypesAndIdsFromRecordedObjects() const;
    const ObjectTypeToVoteMap getVoteSpecifiersForObjectTypes(const std::set<std::string>& objectTypes) const;

    /**
     * Get all reference and non-reference objects for given reference in ism tree.
     *
     * @param Name of reference of ism in the tree for which objects are searched.
     * @return Objects in ism represented as combination of object type and identifier.
     */
    const std::set<std::pair<std::string, std::string> > getObjectTypesAndIdsBelongingToPattern(const std::string& patternName) const;
      
    /**
     * Get ism votes i.e. the scene model for certain objects in a scene.
     *
     * @param patternName Name of scene for which scene model shall be extracted.
     * @param objects Set of scene elements for which a portion of the scene model is to be extracted.
     * @return ISM Votes sorted by scene element.
     */
    const ObjectToVoteMap getVoteSpecifiersForPatternAndObjects(const std::string& patternName, std::set<std::pair<std::string, std::string> > objects) const;

    /**
     * Get all objects of "marker_*" type.
     *
     * @return vector of db IDs mapped to ObjectPtr. (maybe change to set or map)
     */
    const std::vector<std::pair<int, ObjectPtr>> getAllMarkerObjects();

    /**
     * Update the quaternion of the object with dbId
     * @param dbId ID of the object in database
     * @param object ISM object to update the quaternion information from.
     *
     * @return ID of the object in database.
     */
    int updateObjectQuaternion(int dbId, ObjectPtr object);

  };

  typedef boost::shared_ptr<TableHelper> TableHelperPtr;
}
