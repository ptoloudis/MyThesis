"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.postCandidate = exports.postAnswer = exports.postOffer = exports.deleteConnection = exports.createConnection = exports.deleteSession = exports.createSession = exports.getCandidate = exports.getAnswer = exports.getOffer = exports.getConnection = exports.getAll = exports.checkSessionId = exports.reset = void 0;
var offer_1 = require("./offer");
var answer_1 = require("./answer");
var candidate_1 = require("./candidate");
var uuid_1 = require("uuid");
var Disconnection = /** @class */ (function () {
    function Disconnection(id, datetime) {
        this.id = id;
        this.datetime = datetime;
    }
    return Disconnection;
}());
var TimeoutRequestedTime = 10000; // 10sec
var isPrivate;
// [{sessonId:[connectionId,...]}]
var clients = new Map();
// [{sessonId:Date}]
var lastRequestedTime = new Map();
// [{connectionId:[sessionId1, sessionId2]}]
var connectionPair = new Map(); // key = connectionId
// [{sessionId:[{connectionId:Offer},...]}]
var offers = new Map(); // key = sessionId
// [{sessionId:[{connectionId:Answer},...]}]
var answers = new Map(); // key = sessionId
// [{sessionId:[{connectionId:Candidate},...]}]
var candidates = new Map(); // key = sessionId
// [{sessionId:[Disconnection,...]}]
var disconnections = new Map(); // key = sessionId
function getOrCreateConnectionIds(sessionId) {
    var connectionIds = null;
    if (!clients.has(sessionId)) {
        connectionIds = new Set();
        clients.set(sessionId, connectionIds);
    }
    connectionIds = clients.get(sessionId);
    return connectionIds;
}
function reset(mode) {
    isPrivate = mode == "private";
    clients.clear();
    connectionPair.clear();
    offers.clear();
    answers.clear();
    candidates.clear();
    disconnections.clear();
}
exports.reset = reset;
function checkSessionId(req, res, next) {
    if (req.url === '/') {
        next();
        return;
    }
    var id = req.header('session-id');
    if (!clients.has(id)) {
        res.sendStatus(404);
        return;
    }
    lastRequestedTime.set(id, Date.now());
    next();
}
exports.checkSessionId = checkSessionId;
function _deleteConnection(sessionId, connectionId, datetime) {
    clients.get(sessionId).delete(connectionId);
    if (isPrivate) {
        if (connectionPair.has(connectionId)) {
            var pair = connectionPair.get(connectionId);
            var otherSessionId = pair[0] == sessionId ? pair[1] : pair[0];
            if (otherSessionId) {
                if (clients.has(otherSessionId)) {
                    clients.get(otherSessionId).delete(connectionId);
                    var array1 = disconnections.get(otherSessionId);
                    array1.push(new Disconnection(connectionId, datetime));
                }
            }
        }
    }
    else {
        disconnections.forEach(function (array, id) {
            if (id == sessionId)
                return;
            array.push(new Disconnection(connectionId, datetime));
        });
    }
    connectionPair.delete(connectionId);
    offers.get(sessionId).delete(connectionId);
    answers.get(sessionId).delete(connectionId);
    candidates.get(sessionId).delete(connectionId);
    var array2 = disconnections.get(sessionId);
    array2.push(new Disconnection(connectionId, datetime));
}
function _deleteSession(sessionId) {
    if (clients.has(sessionId)) {
        for (var _i = 0, _a = Array.from(clients.get(sessionId)); _i < _a.length; _i++) {
            var connectionId = _a[_i];
            _deleteConnection(sessionId, connectionId, Date.now());
        }
    }
    offers.delete(sessionId);
    answers.delete(sessionId);
    candidates.delete(sessionId);
    clients.delete(sessionId);
    disconnections.delete(sessionId);
}
function _checkForTimedOutSessions() {
    for (var _i = 0, _a = Array.from(clients.keys()); _i < _a.length; _i++) {
        var sessionId = _a[_i];
        if (!lastRequestedTime.has(sessionId))
            continue;
        if (lastRequestedTime.get(sessionId) > Date.now() - TimeoutRequestedTime)
            continue;
        _deleteSession(sessionId);
        console.log("deleted sessionId:".concat(sessionId, " by timeout."));
    }
}
function _getConnection(sessionId) {
    _checkForTimedOutSessions();
    return Array.from(clients.get(sessionId));
}
function _getDisconnection(sessionId, fromTime) {
    _checkForTimedOutSessions();
    var arrayDisconnections = [];
    if (disconnections.size != 0 && disconnections.has(sessionId)) {
        arrayDisconnections = disconnections.get(sessionId);
    }
    if (fromTime > 0) {
        arrayDisconnections = arrayDisconnections.filter(function (v) { return v.datetime >= fromTime; });
    }
    return arrayDisconnections;
}
function _getOffer(sessionId, fromTime) {
    var arrayOffers = [];
    if (offers.size != 0) {
        if (isPrivate) {
            if (offers.has(sessionId)) {
                arrayOffers = Array.from(offers.get(sessionId));
            }
        }
        else {
            var otherSessionMap = Array.from(offers).filter(function (x) { return x[0] != sessionId; });
            arrayOffers = [].concat.apply([], Array.from(otherSessionMap, function (x) { return Array.from(x[1], function (y) { return [y[0], y[1]]; }); }));
        }
    }
    if (fromTime > 0) {
        arrayOffers = arrayOffers.filter(function (v) { return v[1].datetime >= fromTime; });
    }
    return arrayOffers;
}
function _getAnswer(sessionId, fromTime) {
    var arrayAnswers = [];
    if (answers.size != 0 && answers.has(sessionId)) {
        arrayAnswers = Array.from(answers.get(sessionId));
    }
    if (fromTime > 0) {
        arrayAnswers = arrayAnswers.filter(function (v) { return v[1].datetime >= fromTime; });
    }
    return arrayAnswers;
}
function _getCandidate(sessionId, fromTime) {
    var connectionIds = Array.from(clients.get(sessionId));
    var arr = [];
    for (var _i = 0, connectionIds_1 = connectionIds; _i < connectionIds_1.length; _i++) {
        var connectionId = connectionIds_1[_i];
        var pair = connectionPair.get(connectionId);
        if (pair == null) {
            continue;
        }
        var otherSessionId = sessionId === pair[0] ? pair[1] : pair[0];
        if (!candidates.get(otherSessionId) || !candidates.get(otherSessionId).get(connectionId)) {
            continue;
        }
        var arrayCandidates = candidates.get(otherSessionId).get(connectionId)
            .filter(function (v) { return v.datetime >= fromTime; });
        if (arrayCandidates.length === 0) {
            continue;
        }
        for (var _a = 0, arrayCandidates_1 = arrayCandidates; _a < arrayCandidates_1.length; _a++) {
            var candidate = arrayCandidates_1[_a];
            arr.push([connectionId, candidate]);
        }
    }
    return arr;
}
function getAnswer(req, res) {
    // get `fromtime` parameter from request query
    var fromTime = req.query.fromtime ? Number(req.query.fromtime) : 0;
    var sessionId = req.header('session-id');
    var answers = _getAnswer(sessionId, fromTime);
    res.json({ answers: answers.map(function (v) { return ({ connectionId: v[0], sdp: v[1].sdp, type: "answer", datetime: v[1].datetime }); }) });
}
exports.getAnswer = getAnswer;
function getConnection(req, res) {
    // get `fromtime` parameter from request query
    var sessionId = req.header('session-id');
    var connections = _getConnection(sessionId);
    res.json({ connections: connections.map(function (v) { return ({ connectionId: v, type: "connect", datetime: Date.now() }); }) });
}
exports.getConnection = getConnection;
function getOffer(req, res) {
    // get `fromtime` parameter from request query
    var fromTime = req.query.fromtime ? Number(req.query.fromtime) : 0;
    var sessionId = req.header('session-id');
    var offers = _getOffer(sessionId, fromTime);
    res.json({ offers: offers.map(function (v) { return ({ connectionId: v[0], sdp: v[1].sdp, polite: v[1].polite, type: "offer", datetime: v[1].datetime }); }) });
}
exports.getOffer = getOffer;
function getCandidate(req, res) {
    // get `fromtime` parameter from request query
    var fromTime = req.query.fromtime ? Number(req.query.fromtime) : 0;
    var sessionId = req.header('session-id');
    var candidates = _getCandidate(sessionId, fromTime);
    res.json({ candidates: candidates.map(function (v) { return ({ connectionId: v[0], candidate: v[1].candidate, sdpMLineIndex: v[1].sdpMLineIndex, sdpMid: v[1].sdpMid, type: "candidate", datetime: v[1].datetime }); }) });
}
exports.getCandidate = getCandidate;
function getAll(req, res) {
    var fromTime = req.query.fromtime ? Number(req.query.fromtime) : 0;
    var sessionId = req.header('session-id');
    var connections = _getConnection(sessionId);
    var offers = _getOffer(sessionId, fromTime);
    var answers = _getAnswer(sessionId, fromTime);
    var candidates = _getCandidate(sessionId, fromTime);
    var disconnections = _getDisconnection(sessionId, fromTime);
    var datetime = lastRequestedTime.get(sessionId);
    var array = [];
    array = array.concat(connections.map(function (v) { return ({ connectionId: v, type: "connect", datetime: datetime }); }));
    array = array.concat(offers.map(function (v) { return ({ connectionId: v[0], sdp: v[1].sdp, polite: v[1].polite, type: "offer", datetime: v[1].datetime }); }));
    array = array.concat(answers.map(function (v) { return ({ connectionId: v[0], sdp: v[1].sdp, type: "answer", datetime: v[1].datetime }); }));
    array = array.concat(candidates.map(function (v) { return ({ connectionId: v[0], candidate: v[1].candidate, sdpMLineIndex: v[1].sdpMLineIndex, sdpMid: v[1].sdpMid, type: "candidate", datetime: v[1].datetime }); }));
    array = array.concat(disconnections.map(function (v) { return ({ connectionId: v.id, type: "disconnect", datetime: v.datetime }); }));
    array.sort(function (a, b) { return a.datetime - b.datetime; });
    res.json({ messages: array, datetime: datetime });
}
exports.getAll = getAll;
function createSession(req, res) {
    var sessionId = typeof req === "string" ? req : (0, uuid_1.v4)();
    clients.set(sessionId, new Set());
    offers.set(sessionId, new Map());
    answers.set(sessionId, new Map());
    candidates.set(sessionId, new Map());
    disconnections.set(sessionId, []);
    res.json({ sessionId: sessionId });
}
exports.createSession = createSession;
function deleteSession(req, res) {
    var id = req.header('session-id');
    _deleteSession(id);
    res.sendStatus(200);
}
exports.deleteSession = deleteSession;
function createConnection(req, res) {
    var sessionId = req.header('session-id');
    var connectionId = req.body.connectionId;
    var datetime = lastRequestedTime.get(sessionId);
    if (connectionId == null) {
        res.status(400).send({ error: new Error("connectionId is required") });
        return;
    }
    var polite = true;
    if (isPrivate) {
        if (connectionPair.has(connectionId)) {
            var pair = connectionPair.get(connectionId);
            if (pair[0] != null && pair[1] != null) {
                var err = new Error("".concat(connectionId, ": This connection id is already used."));
                console.log(err);
                res.status(400).send({ error: err });
                return;
            }
            else if (pair[0] != null) {
                connectionPair.set(connectionId, [pair[0], sessionId]);
                var map = getOrCreateConnectionIds(pair[0]);
                map.add(connectionId);
            }
        }
        else {
            connectionPair.set(connectionId, [sessionId, null]);
            polite = false;
        }
    }
    var connectionIds = getOrCreateConnectionIds(sessionId);
    connectionIds.add(connectionId);
    res.json({ connectionId: connectionId, polite: polite, type: "connect", datetime: datetime });
}
exports.createConnection = createConnection;
function deleteConnection(req, res) {
    var sessionId = req.header('session-id');
    var connectionId = req.body.connectionId;
    var datetime = lastRequestedTime.get(sessionId);
    _deleteConnection(sessionId, connectionId, datetime);
    res.json({ connectionId: connectionId });
}
exports.deleteConnection = deleteConnection;
function postOffer(req, res) {
    var sessionId = req.header('session-id');
    var connectionId = req.body.connectionId;
    var datetime = lastRequestedTime.get(sessionId);
    var keySessionId = null;
    var polite = false;
    if (isPrivate) {
        if (connectionPair.has(connectionId)) {
            var pair = connectionPair.get(connectionId);
            keySessionId = pair[0] == sessionId ? pair[1] : pair[0];
            if (keySessionId != null) {
                polite = true;
                var map_1 = offers.get(keySessionId);
                map_1.set(connectionId, new offer_1.default(req.body.sdp, datetime, polite));
            }
        }
        res.sendStatus(200);
        return;
    }
    if (!connectionPair.has(connectionId)) {
        connectionPair.set(connectionId, [sessionId, null]);
    }
    keySessionId = sessionId;
    var map = offers.get(keySessionId);
    map.set(connectionId, new offer_1.default(req.body.sdp, datetime, polite));
    res.sendStatus(200);
}
exports.postOffer = postOffer;
function postAnswer(req, res) {
    var sessionId = req.header('session-id');
    var connectionId = req.body.connectionId;
    var datetime = lastRequestedTime.get(sessionId);
    var connectionIds = getOrCreateConnectionIds(sessionId);
    connectionIds.add(connectionId);
    if (!connectionPair.has(connectionId)) {
        res.sendStatus(200);
        return;
    }
    // add connectionPair
    var pair = connectionPair.get(connectionId);
    var otherSessionId = pair[0] == sessionId ? pair[1] : pair[0];
    if (!clients.has(otherSessionId)) {
        // already deleted
        res.sendStatus(200);
        return;
    }
    if (!isPrivate) {
        connectionPair.set(connectionId, [otherSessionId, sessionId]);
    }
    var map = answers.get(otherSessionId);
    map.set(connectionId, new answer_1.default(req.body.sdp, datetime));
    // update datetime for candidates
    var mapCandidates = candidates.get(otherSessionId);
    if (mapCandidates) {
        var arrayCandidates = mapCandidates.get(connectionId);
        if (arrayCandidates) {
            for (var _i = 0, arrayCandidates_2 = arrayCandidates; _i < arrayCandidates_2.length; _i++) {
                var candidate = arrayCandidates_2[_i];
                candidate.datetime = datetime;
            }
        }
    }
    res.sendStatus(200);
}
exports.postAnswer = postAnswer;
function postCandidate(req, res) {
    var sessionId = req.header('session-id');
    var connectionId = req.body.connectionId;
    var datetime = lastRequestedTime.get(sessionId);
    var map = candidates.get(sessionId);
    if (!map.has(connectionId)) {
        map.set(connectionId, []);
    }
    var arr = map.get(connectionId);
    var candidate = new candidate_1.default(req.body.candidate, req.body.sdpMLineIndex, req.body.sdpMid, datetime);
    arr.push(candidate);
    res.sendStatus(200);
}
exports.postCandidate = postCandidate;
//# sourceMappingURL=httphandler.js.map