/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "TableHelper.hpp"
#include <set>
#include <iostream>
#include <vector>
#include <map>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

namespace ISM {

/*******************
      UTIL
  *******************/

TableHelper::TableHelper(std::string dbfilename) {
    this->sqlite.reset(new session(sqlite3, dbfilename.c_str()));
    this->createTablesIfNecessary();
    this->createColumnsIfNecessary();
    objectIdMap_.clear();
}

TableHelper::~TableHelper() {
    (*this->sqlite).close();
}

void TableHelper::createColumnsIfNecessary() {
    std::map<std::string, std::vector<std::pair<std::string,std::string>> > tableNamesToColumns;
    tableNamesToColumns["model_votes"].push_back(std::make_pair("weight", "INTEGER"));
    for(const std::pair<std::string, std::vector<std::pair<std::string,std::string>> >& tablesToColumns : tableNamesToColumns)
    {
        std::vector<std::string> columnsAlreadyExisting;
        std::stringstream infoSs;
        infoSs << "pragma table_info('" << tablesToColumns.first << "');";
        rowset<row> rs = (*sqlite).prepare << infoSs.str();
        for(soci::row& row : rs)
        {
            columnsAlreadyExisting.push_back(row.get<std::string>(1));
        }
        for(const std::pair<std::string,std::string>& col : tablesToColumns.second)
        {
            if(std::find(columnsAlreadyExisting.begin(), columnsAlreadyExisting.end(), col.first)
                    == columnsAlreadyExisting.end())
            {
                std::stringstream alterSs;
                alterSs << "ALTER TABLE " << tablesToColumns.first << " ADD " << col.first <<
                           " " << col.second << ";";
                (*sqlite) << alterSs.str();
            }
        }
    }
}

const std::set<std::string> TableHelper::getObjectsInPattern(std::string pattern_name){
    rowset<row> rs = ((*sqlite).prepare<<"SELECT DISTINCT ro.type, ro.observedID FROM recorded_objects ro JOIN recorded_poses rpo on ro.id = rpo.objectId JOIN recorded_sets rs on rpo.setId = rs.id JOIN recorded_patterns rpa on rs.patternId = rpa.id where rpa.name = '" << pattern_name << "';");

    std::set<std::string> types;
    BOOST_FOREACH(row const& row, rs) {
        types.insert(row.get<std::string>(0) + row.get<std::string>(1));
    }
    return types;
}

void TableHelper::createTablesIfNecessary() const {
    std::set<std::string> tables;

    rowset<row> rs = (*sqlite).prepare<< "SELECT tbl_name FROM sqlite_master WHERE type = 'table';";
    for (rowset<row>::const_iterator it = rs.begin(); it != rs.end(); ++it) {
        row const& row = *it;
        tables.insert(row.get<std::string>(0));
    }

    std::map<std::string, std::string> tableDefs;
    tableDefs["recorded_objects"] = "id INTEGER PRIMARY KEY, type TEXT, observedId TEXT, resourcePath TEXT";
    tableDefs["recorded_poses"] = "id INTEGER PRIMARY KEY, objectId INTEGER, setId INTEGER, px FLOAT, py FLOAT, pz FLOAT, qw FLOAT, qx FLOAT, qy FLOAT, qz FLOAT";
    tableDefs["recorded_sets"] = "id INTEGER PRIMARY KEY, patternId INTEGER";
    tableDefs["recorded_patterns"] = "id INTEGER PRIMARY KEY, name TEXT UNIQUE";
    tableDefs["model_objects"] = "id INTEGER PRIMARY KEY, type TEXT UNIQUE";
    tableDefs["model_votes"] =
            "id INTEGER PRIMARY KEY, objectId INTEGER, patternId INTEGER, observedId TEXT, radius FLOAT"
            ", qw FLOAT, qx FLOAT, qy FLOAT, qz FLOAT"
            ", qw2 FLOAT, qx2 FLOAT, qy2 FLOAT, qz2 FLOAT"
            ", qpw FLOAT, qpx FLOAT, qpy FLOAT, qpz FLOAT"
            ", qpw2 FLOAT, qpx2 FLOAT, qpy2 FLOAT, qpz2 FLOAT"
            ", trackIndex INTEGER, weight INTEGER";
    tableDefs["model_patterns"] = "id INTEGER PRIMARY KEY, name TEXT UNIQUE, expectedMaxWeight INTEGER";

    typedef std::pair<std::string, std::string> pair_type;
    BOOST_FOREACH(pair_type p, tableDefs) {
        if (tables.find(p.first) == tables.end()) {
            this->createTable(p.first, p.second);
        }
    }
}

void TableHelper::createTable(const std::string& tablename, const std::string& sql) const {
    try {
        (*sqlite).once<<"CREATE TABLE `"<<tablename<<"` ("<<sql<<");";
    } catch (soci_error e) {
        std::cerr<<"SQL error "<<e.what()<<std::endl;
        throw e;
    }
}

void TableHelper::dropTable(const std::string& tablename) const {
    try {
        (*sqlite).once<<"DROP TABLE `"<<tablename<<"`;";
    } catch (soci_error e) {
        std::cerr<<"SQL error "<<e.what()<<std::endl;
        throw e;
    }
}

void TableHelper::dropRecordTables() const
{
    this->dropTable("recorded_objects");
    this->dropTable("recorded_poses");
    this->dropTable("recorded_patterns");
    this->dropTable("recorded_sets");
}

void TableHelper::dropTables() const
{
    this->dropTable("recorded_objects");
    this->dropTable("recorded_poses");
    this->dropTable("recorded_patterns");
    this->dropTable("recorded_sets");
    this->dropTable("model_votes");
    this->dropTable("model_patterns");
    this->dropTable("model_objects");
}

void TableHelper::dropModelTables() const {
    this->dropTable("model_votes");
    this->dropTable("model_patterns");
    this->dropTable("model_objects");
}

int TableHelper::getLastInsertId(const std::string& tablename) const {
    int id = 0;
    try {
        (*sqlite)<<"SELECT id FROM `"<<tablename<<"` ORDER BY ID DESC LIMIT 1;", into(id);
    } catch (soci_error e) {
        std::cerr<<"SQL error "<<e.what()<<std::endl;
        throw e;
    }

    return id;
}

/*******************
      Recorded
  *******************/
int TableHelper::insertRecordedObjectIfNecessary(ObjectPtr object)
{
    std::pair<std::string, std::string> objectIdentifier = std::make_pair(object->type, object->observedId);
    if(objectIdMap_.find(objectIdentifier) != objectIdMap_.end())
    {
     return objectIdMap_[objectIdentifier];
    }

    int id;
    indicator ind;
    (*sqlite) << "SELECT id FROM `recorded_objects` where type = :type and observedId = :observedId;", into(id, ind), use(object->type),use(object->observedId);
    if ((*sqlite).got_data() && ind == i_ok) {
        objectIdMap_[objectIdentifier] = id;
        return id;
    } else {
        int lastInsertId = getLastInsertId("recorded_objects");
        (*sqlite) << "INSERT INTO `recorded_objects` (id, type, observedId, resourcePath) values (:id, :type, :oid, :resourcePath);",
                use(lastInsertId+1),
                use(object->type),
                use(object->observedId),
                use(object->ressourcePath.string());
        objectIdMap_[objectIdentifier] = lastInsertId + 1;
        return lastInsertId + 1;
    }
}


int TableHelper::insertRecordedObject(const boost::shared_ptr<Object>& o, int setId) {
    int objectId = this->insertRecordedObjectIfNecessary(o);
    (*sqlite) << "INSERT INTO `recorded_poses` (objectId, setId, px, py, pz, qw, qx, qy, qz) values (:objectId, :setId, :px, :py, :pz, :qw, :qx, :qy, :qz);",
            use(objectId),
            use(setId),
            use(o->pose->point->eigen.x()),
            use(o->pose->point->eigen.y()),
            use(o->pose->point->eigen.z()),
            use(o->pose->quat->eigen.w()),
            use(o->pose->quat->eigen.x()),
            use(o->pose->quat->eigen.y()),
            use(o->pose->quat->eigen.z());

    return this->getLastInsertId("recorded_objects");
}

int TableHelper::insertRecordedObjectSet(const ObjectSetPtr& os, const std::string& patternName) {
    std::vector<boost::shared_ptr<Object> > objects = os->objects;

    transaction trans(*sqlite);
    int patternId = this->ensureRecordedPatternName(patternName);

    (*sqlite) << "INSERT INTO `recorded_sets` (patternId) VALUES (:patternId);", use(patternId);
    int setId = this->getLastInsertId("recorded_sets");
    for (size_t i = 0; i < objects.size(); i++) {
        this->insertRecordedObject(objects[i], setId);
    }

    trans.commit();

    return setId;
}

int TableHelper::insertRecordedPattern(const std::string& patternName) const {
    (*sqlite) << "INSERT INTO `recorded_patterns` (name) VALUES (:patternName);", use(patternName);
    return this->getLastInsertId("recorded_patterns");
}

int TableHelper::ensureRecordedPatternName(const std::string& patternName) const {
    int id = this->getRecordedPatternId(patternName);
    return id == 0 ? this->insertRecordedPattern(patternName) : id;
}

std::vector<std::string> TableHelper::getRecordedPatternNames() const {
    std::vector<std::string> names;
    rowset<std::string> rs = ((*sqlite).prepare<< "SELECT name FROM `recorded_patterns`;");
    for (std::string& name : rs) {
        names.push_back(name);;
    }
    return names;
}

int TableHelper::getRecordedPatternId(const std::string& patternName) const {
    int id;
    indicator ind;
    (*sqlite) << "SELECT id FROM `recorded_patterns` WHERE name = :name;", into(id, ind), use(patternName);
    if ((*sqlite).got_data() && ind == i_ok) {
        return id;
    } else {
        return 0;
    }
}

const RecordedPatternPtr TableHelper::getRecordedPattern(const std::string& patternName) const {
    boost::shared_ptr<RecordedPattern> pattern;
    int patternId = this->getRecordedPatternId(patternName);

    if (patternId != 0) {
        pattern.reset(new RecordedPattern(patternName));

        rowset<int> rs = ((*sqlite).prepare<< "SELECT id FROM `recorded_sets` WHERE patternId = :patternId;", use(patternId));
        for (rowset<int>::const_iterator it = rs.begin(); it != rs.end(); ++it) {
            pattern->addObjectSet(this->getRecordedObjectSet(*it));
        }
    }

    return pattern;
}

const std::vector<int> TableHelper::getSetIds() const {
    std::vector<int> setIds;
    rowset<int> rs = ((*sqlite).prepare<< "SELECT DISTINCT setId FROM `recorded_poses`;");

    for (rowset<int>::const_iterator it = rs.begin(); it != rs.end(); ++it) {
        setIds.push_back(*it);
    }
    return setIds;
}

const ObjectSetPtr TableHelper::getRecordedObjectSet(int setId) const {
    boost::shared_ptr<ObjectSet> s(new ObjectSet());

    rowset<row> rs = ((*sqlite).prepare<<
                      "SELECT ro.type, ro.observedId, rp.px, rp.py, rp.pz, rp.qw, rp.qx, rp.qy, rp.qz, ro.resourcePath FROM `recorded_objects` ro JOIN `recorded_poses` rp WHERE ro.id = rp.objectId AND setId = :setId;",
                      use(setId)
                      );

    for (rowset<row>::const_iterator it = rs.begin(); it != rs.end(); ++it) {
        row const& row = *it;
        boost::filesystem::path meshPath = row.get<std::string>(9,"");
        ObjectPtr object(
                    new Object(
                        row.get<std::string>(0, ""),
                        PosePtr(new Pose(
                                    new Point(
                                        row.get<double>(2, 0.0),
                                        row.get<double>(3, 0.0),
                                        row.get<double>(4, 0.0)
                                        ),
                                    new Quaternion(
                                        row.get<double>(5, 0.0),
                                        row.get<double>(6, 0.0),
                                        row.get<double>(7, 0.0),
                                        row.get<double>(8, 0.0)
                                        )
                                    )
                                ),
                        row.get<std::string>(1, "")
                        )
                    );
        object->ressourcePath = meshPath;
        s->insert(object);
    }
    return s;
}

/*******************
      Model
  *******************/

int TableHelper::insertModelVoteSpecifier(const VoteSpecifierPtr& vote) const {
    int patternId = this->ensureModelPatternName(vote->patternName);
    int objectId = this->ensureModelObjectType(vote->objectType);

    (*sqlite) << "INSERT INTO `model_votes` "<<
                 "(objectId, patternId, observedId, radius, qw, qx, qy, qz, qw2, qx2, qy2, qz2, qpw, qpx, qpy, qpz, qpw2, qpx2, qpy2, qpz2, trackIndex, weight) values "<<
                 "(:objectId, :patternId, :observedId, :radius, :qw, :qx, :qy, :qz, :qw2, :qx2, :qy2, :qz2, :qpw, :qpx, :qpy, :qpz, :qpw2, :qpx2, :qpy2, :qpz2, :trackIndex, :weight);",
            use(objectId),
            use(patternId),
            use(vote->observedId),
            use(vote->radius),
            use(vote->objectToRefQuat->eigen.w()),
            use(vote->objectToRefQuat->eigen.x()),
            use(vote->objectToRefQuat->eigen.y()),
            use(vote->objectToRefQuat->eigen.z()),
            use(vote->objectToRefPoseQuat->eigen.w()),
            use(vote->objectToRefPoseQuat->eigen.x()),
            use(vote->objectToRefPoseQuat->eigen.y()),
            use(vote->objectToRefPoseQuat->eigen.z()),
            use(vote->refToObjectQuat->eigen.w()),
            use(vote->refToObjectQuat->eigen.x()),
            use(vote->refToObjectQuat->eigen.y()),
            use(vote->refToObjectQuat->eigen.z()),
            use(vote->refToObjectPoseQuat->eigen.w()),
            use(vote->refToObjectPoseQuat->eigen.x()),
            use(vote->refToObjectPoseQuat->eigen.y()),
            use(vote->refToObjectPoseQuat->eigen.z()),
            use(vote->trackIndex),
            use(vote->weight);

    return this->getLastInsertId("model_votes");
}

int TableHelper::ensureModelPatternName(const std::string& patternName) const {
    int id = this->getModelPatternId(patternName);
    return id == 0 ? this->insertModelPattern(patternName) : id;
}

int TableHelper::ensureModelObjectType(const std::string& objectType) const {
    int id = this->getModelObjectTypeId(objectType);
    return id == 0 ? this->insertModelObjectType(objectType) : id;
}

int TableHelper::insertModelPattern(const std::string& patternName) const {
    (*sqlite) << "INSERT INTO `model_patterns` (name) VALUES (:patternName);", use(patternName);
    return this->getLastInsertId("model_patterns");
}

int TableHelper::upsertModelPattern(const std::string& patternName, int expectedMaxWeight) const {
    int patternId = this->getModelPatternId(patternName);
    if (patternId == 0) {
        (*sqlite) << "INSERT INTO `model_patterns` (name, expectedMaxWeight) VALUES (:patternName, :expectedMaxWeight);",
                use(patternName),
                use(expectedMaxWeight);
    } else {
        (*sqlite) << "REPLACE INTO `model_patterns` (id, name, expectedMaxWeight) VALUES (:id, :patternName, :expectedMaxWeight);",
                use(patternId),
                use(patternName),
                use(expectedMaxWeight);
    }
    return this->getLastInsertId("model_patterns");
}

int TableHelper::insertModelObjectType(const std::string& objectType) const {
    (*sqlite) << "INSERT INTO `model_objects` (type) VALUES (:objectType);", use(objectType);
    return this->getLastInsertId("model_objects");
}

int TableHelper::getModelPatternId(const std::string& patternName) const {
    int id;
    indicator ind;
    (*sqlite) << "SELECT id FROM `model_patterns` WHERE name = :name;", into(id, ind), use(patternName);
    if ((*sqlite).got_data() && ind == i_ok) {
        return id;
    } else {
        return 0;
    }
}

int TableHelper::getModelObjectTypeId(const std::string& objectType) const {
    int id;
    indicator ind;
    (*sqlite) << "SELECT id FROM `model_objects` WHERE type = :objectType;", into(id, ind), use(objectType);
    if ((*sqlite).got_data() && ind == i_ok) {
        return id;
    } else {
        return 0;
    }
}

std::vector<std::string> TableHelper::getModelPatternNames() const {
    std::vector<std::string> names;
    rowset<std::string> rs = ((*sqlite).prepare<< "SELECT name FROM `model_patterns`;");
    for (std::string& name : rs) {
        names.push_back(name);;
    }

    return names;
}

const PatternNameToPatternMap TableHelper::getPatternDefinitionsByName(const std::set<std::string>& patternNames) const {
    PatternNameToPatternMap patterns;
    BOOST_FOREACH(std::string patternName, patternNames) {
        int expectedMaxWeight;
        indicator indEOC;
        (*sqlite) <<
                     "SELECT expectedMaxWeight "<<
                     "FROM `model_patterns` "<<
                     "WHERE name = :patternName LIMIT 1;",
                into(expectedMaxWeight, indEOC),
                use(patternName);
        if ((*sqlite).got_data() && indEOC == i_ok) {
            patterns[patternName] = PatternPtr(new Pattern(patternName, expectedMaxWeight));
        }
    }

    return patterns;
}

const std::set<std::string> TableHelper::getObjectTypes() const {
    rowset<row> rs = ((*sqlite).prepare<<"SELECT type FROM `model_objects`;");
    std::set<std::string> types;
    BOOST_FOREACH(row const& row, rs) {
        types.insert(row.get<std::string>(0));
    }
    return types;
}

const std::map<std::string, boost::filesystem::path> TableHelper::getRessourcePaths() const {
    rowset<row> rs = ((*sqlite).prepare<<"SELECT DISTINCT type, resourcePath FROM `recorded_objects`;");
    std::map<std::string, boost::filesystem::path> types_to_ressource_paths_map;
    BOOST_FOREACH(row const& row, rs) {
        std::string type = row.get<std::string>(0);
        boost::filesystem::path meshPath = row.get<std::string>(1,"");
        types_to_ressource_paths_map[type] = meshPath;
    }
    return types_to_ressource_paths_map;
}

const std::map<std::string, std::map<std::string, std::string>> TableHelper::getModelWeightsPerTypeAndId() const {
    std::map<std::string, std::map<std::string, std::string>> modelWeightsPerTypeAndId;
    try {
        rowset<row> rs = ((*sqlite).prepare<<"SELECT type, observedId, weight FROM `model_weight`;");
        BOOST_FOREACH(row const& row, rs) {
            modelWeightsPerTypeAndId[row.get<std::string>(0)][row.get<std::string>(1)] = row.get<std::string>(2);
        }
    } catch (soci::sqlite3_soci_error e)  {
        if (std::string(e.what()).find("no such table: model_weight") != std::string::npos) {
            // the table model_weight will be added by intermediate_object_generator; if not present return empty map
        } else {
            throw e;
        }
    }
    return modelWeightsPerTypeAndId;
}

const std::set<std::pair<std::string, std::string> > TableHelper::getObjectTypesAndIdsFromModelObjects() const {
    rowset<row> rs = ((*sqlite).prepare<<"SELECT DISTINCT type, observedId FROM model_votes v JOIN model_objects o on o.id = v.objectId");
    std::set<std::pair<std::string, std::string> > typesAndIds;
    BOOST_FOREACH(row const& row, rs) {
        typesAndIds.insert(std::make_pair(row.get<std::string>(0), row.get<std::string>(1)));
    }
    return typesAndIds;
}

const std::set<std::pair<std::string, std::string> > TableHelper::getObjectTypesAndIdsFromRecordedObjects() const {
    rowset<row> rs = ((*sqlite).prepare<<"SELECT DISTINCT type, observedId FROM `recorded_objects`;");
    std::set<std::pair<std::string, std::string> > typesAndIds;
    BOOST_FOREACH(row const& row, rs) {
        typesAndIds.insert(std::make_pair(row.get<std::string>(0), row.get<std::string>(1)));
    }
    return typesAndIds;
}

const ObjectTypeToVoteMap TableHelper::getVoteSpecifiersForObjectTypes(const std::set<std::string>& objectTypes) const {
    ObjectTypeToVoteMap voteSpecifierMap;
    BOOST_FOREACH(std::string objectType, objectTypes) {
        int objectId = getModelObjectTypeId(objectType);
        rowset<row> rs = ((*sqlite).prepare<<
                          "SELECT radius, name, type, observedId, qw, qx, qy, qz, qw2, qx2, qy2, qz2, qpw, qpx, qpy, qpz, qpw2, qpx2, qpy2, qpz2, trackIndex, v.weight "<<
                          "FROM `model_votes` AS v "<<
                          "JOIN `model_objects` AS o ON v.objectId = o.id "<<
                          "JOIN `model_patterns` AS p ON v.patternId = p.id "<<
                          "WHERE objectId = :objectId;",
                          use(objectId)
                          );
        std::vector<VoteSpecifierPtr> specifiers;
        BOOST_FOREACH(row const& row, rs) {
            VoteSpecifierPtr vote(
                        new VoteSpecifier(
                            QuaternionPtr(
                                new Quaternion(
                                    row.get<double>(4, 0.0),
                                    row.get<double>(5, 0.0),
                                    row.get<double>(6, 0.0),
                                    row.get<double>(7, 0.0)
                                    )
                                ),
                            QuaternionPtr(
                                new Quaternion(
                                    row.get<double>(8, 0.0),
                                    row.get<double>(9, 0.0),
                                    row.get<double>(10, 0.0),
                                    row.get<double>(11, 0.0)
                                    )
                                ),
                            QuaternionPtr(
                                new Quaternion(
                                    row.get<double>(12, 0.0),
                                    row.get<double>(13, 0.0),
                                    row.get<double>(14, 0.0),
                                    row.get<double>(15, 0.0)
                                    )
                                ),
                            QuaternionPtr(
                                new Quaternion(
                                    row.get<double>(16, 0.0),
                                    row.get<double>(17, 0.0),
                                    row.get<double>(18, 0.0),
                                    row.get<double>(19, 0.0)
                                    )
                                ),
                            row.get<double>(0, 0.0),
                            row.get<std::string>(1, ""),
                            row.get<std::string>(2, ""),
                            row.get<std::string>(3, ""),
                            row.get<int>(20, 0)
                            )
                        );
            vote->weight = row.get<int>(21, 1);
            specifiers.push_back(vote);
        }
        voteSpecifierMap[objectType] = specifiers;
    };

    return voteSpecifierMap;
}

const ObjectToVoteMap TableHelper::getVoteSpecifiersForPatternAndObjects(const std::string& patternName, std::set<std::pair<std::string, std::string> > objects) const{

    ObjectToVoteMap voteSpecifierMap;
    int patternId = getModelPatternId(patternName);

    for(std::pair<std::string, std::string> object : objects) {

        int objectId = getModelObjectTypeId(object.first);
        std::string observedId = object.second;

        rowset<row> rs = ((*sqlite).prepare<<
                          "SELECT radius, name, type, observedId, qw, qx, qy, qz, qw2, qx2, qy2, qz2, qpw, qpx, qpy, qpz, qpw2, qpx2, qpy2, qpz2, trackIndex, v.weight "<<
                          "FROM `model_votes` AS v "<<
                          "JOIN `model_objects` AS o ON v.objectId = o.id "<<
                          "JOIN `model_patterns` AS p ON v.patternId = p.id "<<
                          "WHERE objectId = :objectId AND patternId = :patternId AND observedId = :observedId;",
                          use(objectId), use(patternId), use(observedId)
                          );
        std::vector<VoteSpecifierPtr> specifiers;
        BOOST_FOREACH(row const& row, rs) {
            VoteSpecifierPtr vote(
                        new VoteSpecifier(
                            QuaternionPtr(
                                new Quaternion(
                                    row.get<double>(4, 0.0),
                                    row.get<double>(5, 0.0),
                                    row.get<double>(6, 0.0),
                                    row.get<double>(7, 0.0)
                                    )
                                ),
                            QuaternionPtr(
                                new Quaternion(
                                    row.get<double>(8, 0.0),
                                    row.get<double>(9, 0.0),
                                    row.get<double>(10, 0.0),
                                    row.get<double>(11, 0.0)
                                    )
                                ),
                            QuaternionPtr(
                                new Quaternion(
                                    row.get<double>(12, 0.0),
                                    row.get<double>(13, 0.0),
                                    row.get<double>(14, 0.0),
                                    row.get<double>(15, 0.0)
                                    )
                                ),
                            QuaternionPtr(
                                new Quaternion(
                                    row.get<double>(16, 0.0),
                                    row.get<double>(17, 0.0),
                                    row.get<double>(18, 0.0),
                                    row.get<double>(19, 0.0)
                                    )
                                ),
                            row.get<double>(0, 0.0),
                            row.get<std::string>(1, ""),
                            row.get<std::string>(2, ""),
                            row.get<std::string>(3, ""),
                            row.get<int>(20, 0)
                            )
                        );

            vote->weight = row.get<int>(21, 1);
            specifiers.push_back(vote);
        }

        if(!specifiers.empty())
            voteSpecifierMap[object.first][object.second] = specifiers;
        else
            std::cout << "Object (" << object.first << "," << object.second << ") and pattern " << patternName <<  " is an invalid combination." << std::endl;

    }

    return voteSpecifierMap;

}

const std::set<std::pair<std::string, std::string> > TableHelper::getObjectTypesAndIdsBelongingToPattern(const std::string& patternName) const {

    int patternId = getModelPatternId(patternName);

    rowset<row> rs = ((*sqlite).prepare<<"SELECT DISTINCT type, observedId " <<
                      "FROM `model_votes` AS v "<<
                      "JOIN `model_objects` AS o ON v.objectId = o.id "<<
                      "JOIN `model_patterns` AS p ON v.patternId = p.id "<<
                      "WHERE patternId = :patternId;",
                      use(patternId));

    std::set<std::pair<std::string, std::string> > typesAndIds;
    BOOST_FOREACH(row const& row, rs) {
        typesAndIds.insert(std::make_pair(row.get<std::string>(0), row.get<std::string>(1)));
    }
    return typesAndIds;
}

const std::vector<std::pair<int, ObjectPtr>> TableHelper::getAllMarkerObjects() {
                                             rowset<row> rs = ((*sqlite).prepare << "SELECT type, observedId, px, py, pz, qw, qx, qy, qz, resourcePath, id FROM `recorded_objects` where type like 'marker_%'");
                                             std::vector<std::pair<int, ObjectPtr>> result;
                                             for (rowset<row>::const_iterator it = rs.begin(); it != rs.end(); ++it) {
    row const& row = *it;
    boost::filesystem::path meshPath = row.get<std::string>(9,"");
    ObjectPtr object(
                new Object(
                    row.get<std::string>(0, ""),
                    PosePtr(new Pose(
                                new Point(
                                    row.get<double>(2, 0.0),
                                    row.get<double>(3, 0.0),
                                    row.get<double>(4, 0.0)
                                    ),
                                new Quaternion(
                                    row.get<double>(5, 0.0),
                                    row.get<double>(6, 0.0),
                                    row.get<double>(7, 0.0),
                                    row.get<double>(8, 0.0)
                                    )
                                )
                            ),
                    row.get<std::string>(1, "")
                    )
                );
    object->ressourcePath = meshPath;
    result.push_back(std::make_pair(row.get<int>(10, 0), object));
}
return result;
}

int TableHelper::updateObjectQuaternion(int dbId, ObjectPtr object) {
    QuaternionPtr quat = object->pose->quat;
    double qw = quat->eigen.w();
    double qx = quat->eigen.x();
    double qy = quat->eigen.y();
    double qz = quat->eigen.z();
    transaction trans(*sqlite);
    (*sqlite)<< "UPDATE `recorded_poses` SET qw = :qw , qx = :qx , qy = :qy , qz = :qz WHERE id = :dbId;",
                 use(qw), use(qx), use(qy), use(qz), use(dbId);
    trans.commit();
    return dbId;

}
}

