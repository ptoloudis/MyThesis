"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g = Object.create((typeof Iterator === "function" ? Iterator : Object).prototype);
    return g.next = verb(0), g["throw"] = verb(1), g["return"] = verb(2), typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (g && (g = 0, op[0] && (_ = 0)), _) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
Object.defineProperty(exports, "__esModule", { value: true });
var express_1 = require("@jest-mock/express");
var httpHandler = require("../src/class/httphandler");
var RetriesToForceTimeout = 11; // Waits a second each time, timeout is 10 sec for httphandler.
describe('http signaling test in public mode', function () {
    var sessionId = "abcd1234";
    var sessionId2 = "abcd5678";
    var sessionId3 = "abcd9101112";
    var connectionId = "12345";
    var connectionId2 = "67890";
    var testsdp = "test sdp";
    var _a = (0, express_1.getMockRes)(), res = _a.res, next = _a.next, mockClear = _a.mockClear;
    var req = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId; }) });
    var req2 = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId2; }) });
    var req3 = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId3; }) });
    beforeAll(function () {
        httpHandler.reset("public");
    });
    beforeEach(function () {
        mockClear();
        httpHandler.checkSessionId(req, res, next);
        httpHandler.checkSessionId(req2, res, next);
    });
    test('throw check has session', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            httpHandler.checkSessionId(req, res, next);
            expect(res.sendStatus).toHaveBeenCalledWith(404);
            expect(next).not.toHaveBeenCalled();
            return [2 /*return*/];
        });
    }); });
    test('create session', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.createSession(sessionId, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ sessionId: sessionId });
                    return [2 /*return*/];
            }
        });
    }); });
    test('create session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.createSession(sessionId2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ sessionId: sessionId2 });
                    return [2 /*return*/];
            }
        });
    }); });
    test('create connection from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId };
                    req.body = body;
                    return [4 /*yield*/, httpHandler.createConnection(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connectionId: connectionId, polite: true, datetime: expect.anything(), type: "connect" });
                    return [2 /*return*/];
            }
        });
    }); });
    test('create connection from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId2 };
                    req2.body = body;
                    return [4 /*yield*/, httpHandler.createConnection(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connectionId: connectionId2, polite: true, datetime: expect.anything(), type: "connect" });
                    return [2 /*return*/];
            }
        });
    }); });
    test('get connection from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var connect;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getConnection(req, res)];
                case 1:
                    _a.sent();
                    connect = { connectionId: connectionId, datetime: expect.anything(), type: "connect" };
                    expect(res.json).toHaveBeenCalledWith({ connections: expect.arrayContaining([connect]) });
                    return [2 /*return*/];
            }
        });
    }); });
    test('get all from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var connect;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 1:
                    _a.sent();
                    connect = { connectionId: connectionId, datetime: expect.anything(), type: "connect" };
                    expect(res.json).toHaveBeenCalledWith({ messages: expect.arrayContaining([connect]), datetime: expect.anything() });
                    return [2 /*return*/];
            }
        });
    }); });
    test('post offer from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer" };
                    req.body = body;
                    return [4 /*yield*/, httpHandler.postOffer(req, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('get offer from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getOffer(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ offers: [] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('get offer from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getOffer(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ offers: [{ connectionId: connectionId, sdp: testsdp, polite: false, datetime: expect.anything(), type: "offer" }] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('post answer from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId, sdp: testsdp };
                    req2.body = body;
                    return [4 /*yield*/, httpHandler.postAnswer(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('get answer from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getAnswer(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ answers: [{ connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "answer" }] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('get answer from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getAnswer(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ answers: [] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('post candidate from sesson1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId, candidate: "testcandidate", sdpMLineIndex: 0, sdpMid: 0 };
                    req.body = body;
                    return [4 /*yield*/, httpHandler.postCandidate(req, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('get candidate from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getCandidate(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ candidates: [] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('get candidate from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getCandidate(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ candidates: [{ connectionId: connectionId, candidate: "testcandidate", sdpMLineIndex: 0, sdpMid: 0, type: "candidate", datetime: expect.anything() }] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('delete connection from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId };
                    req2.body = body;
                    return [4 /*yield*/, httpHandler.deleteConnection(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connectionId: connectionId });
                    return [2 /*return*/];
            }
        });
    }); });
    test('disconnection get from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var disconnect;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 1:
                    _a.sent();
                    disconnect = { connectionId: connectionId, datetime: expect.anything(), type: "disconnect" };
                    expect(res.json).toHaveBeenCalledWith({ messages: expect.arrayContaining([disconnect]), datetime: expect.anything() });
                    return [2 /*return*/];
            }
        });
    }); });
    test('delete connection from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId };
                    req.body = body;
                    return [4 /*yield*/, httpHandler.deleteConnection(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connectionId: connectionId });
                    return [2 /*return*/];
            }
        });
    }); });
    test('delete session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var req;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    req = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId; }) });
                    return [4 /*yield*/, httpHandler.deleteSession(req, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('delete session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        var req2;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    req2 = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId2; }) });
                    return [4 /*yield*/, httpHandler.deleteSession(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('disconnection get when session2 disconnects before session1 answer', function () { return __awaiter(void 0, void 0, void 0, function () {
        var connectBody, offerBody, offer, deleteBody, answerBody, disconnect;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    httpHandler.reset("public");
                    return [4 /*yield*/, httpHandler.createSession(sessionId, res)];
                case 1:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.createSession(sessionId2, res)];
                case 2:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 3:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: [], datetime: expect.anything() });
                    connectBody = { connectionId: connectionId };
                    req.body = connectBody;
                    return [4 /*yield*/, httpHandler.createConnection(req, res)];
                case 4:
                    _a.sent();
                    offerBody = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer" };
                    req.body = offerBody;
                    return [4 /*yield*/, httpHandler.postOffer(req, res)];
                case 5:
                    _a.sent();
                    offer = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer", polite: false };
                    return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 6:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.not.arrayContaining([offer]), datetime: expect.anything() });
                    return [4 /*yield*/, httpHandler.getAll(req2, res)];
                case 7:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.arrayContaining([offer]), datetime: expect.anything() });
                    deleteBody = { connectionId: connectionId };
                    req2.body = deleteBody;
                    return [4 /*yield*/, httpHandler.deleteConnection(req, res)];
                case 8:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.deleteSession(req, res)];
                case 9:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenLastCalledWith(200);
                    answerBody = { connectionId: connectionId, sdp: testsdp };
                    req2.body = answerBody;
                    return [4 /*yield*/, httpHandler.postAnswer(req2, res)];
                case 10:
                    _a.sent();
                    disconnect = { connectionId: connectionId, type: "disconnect", datetime: expect.anything() };
                    return [4 /*yield*/, httpHandler.getAll(req2, res)];
                case 11:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.arrayContaining([disconnect]), datetime: expect.anything() });
                    return [4 /*yield*/, httpHandler.deleteSession(req2, res)];
                case 12:
                    _a.sent();
                    return [2 /*return*/];
            }
        });
    }); });
    test('Timed out session2 deleted after session1 resends offer', function () { return __awaiter(void 0, void 0, void 0, function () {
        var connectBody, offerBody, offer, answerBody, i;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    httpHandler.reset("public");
                    return [4 /*yield*/, httpHandler.createSession(sessionId, res)];
                case 1:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.createSession(sessionId2, res)];
                case 2:
                    _a.sent();
                    req.url = "";
                    req2.url = "";
                    return [4 /*yield*/, httpHandler.checkSessionId(req, res, next)];
                case 3:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.checkSessionId(req2, res, next)];
                case 4:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 5:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: [], datetime: expect.anything() });
                    connectBody = { connectionId: connectionId };
                    req.body = connectBody;
                    return [4 /*yield*/, httpHandler.createConnection(req, res)];
                case 6:
                    _a.sent();
                    offerBody = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer" };
                    req.body = offerBody;
                    return [4 /*yield*/, httpHandler.postOffer(req, res)];
                case 7:
                    _a.sent();
                    offer = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer", polite: false };
                    return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 8:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.not.arrayContaining([offer]), datetime: expect.anything() });
                    return [4 /*yield*/, httpHandler.getAll(req2, res)];
                case 9:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.arrayContaining([offer]), datetime: expect.anything() });
                    answerBody = { connectionId: connectionId, sdp: testsdp };
                    req2.body = answerBody;
                    return [4 /*yield*/, httpHandler.postAnswer(req2, res)];
                case 10:
                    _a.sent();
                    // resend offer after answer to simulate PeerCandidate entering into failed state
                    req.body = offerBody;
                    return [4 /*yield*/, httpHandler.postOffer(req, res)];
                case 11:
                    _a.sent();
                    i = 0;
                    _a.label = 12;
                case 12:
                    if (!(i < RetriesToForceTimeout + 1)) return [3 /*break*/, 16];
                    return [4 /*yield*/, httpHandler.checkSessionId(req, res, next)];
                case 13:
                    _a.sent();
                    return [4 /*yield*/, new Promise(function (resolve) { return setTimeout(resolve, 1000); })];
                case 14:
                    _a.sent();
                    _a.label = 15;
                case 15:
                    ++i;
                    return [3 /*break*/, 12];
                case 16: 
                // Get all for session1 to trigger cleaning up associated session that timed out.
                return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 17:
                    // Get all for session1 to trigger cleaning up associated session that timed out.
                    _a.sent();
                    // Check that we do have session1 still
                    return [4 /*yield*/, httpHandler.checkSessionId(req, res, next)];
                case 18:
                    // Check that we do have session1 still
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenLastCalledWith(200);
                    // Check that we no longer have session2
                    return [4 /*yield*/, httpHandler.checkSessionId(req2, res, next)];
                case 19:
                    // Check that we no longer have session2
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenLastCalledWith(404);
                    return [4 /*yield*/, httpHandler.deleteSession(req, res)];
                case 20:
                    _a.sent();
                    return [2 /*return*/];
            }
        });
    }); }, 16000);
    test('Timed out sessions are deleted when other sessions check', function () { return __awaiter(void 0, void 0, void 0, function () {
        var connectBody, offerBody, offer, answerBody, i;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    httpHandler.reset("public");
                    return [4 /*yield*/, httpHandler.createSession(sessionId, res)];
                case 1:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.createSession(sessionId2, res)];
                case 2:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.createSession(sessionId3, res)];
                case 3:
                    _a.sent();
                    req.url = "";
                    req2.url = "";
                    req3.url = "";
                    return [4 /*yield*/, httpHandler.checkSessionId(req, res, next)];
                case 4:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.checkSessionId(req2, res, next)];
                case 5:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.checkSessionId(req3, res, next)];
                case 6:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 7:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: [], datetime: expect.anything() });
                    connectBody = { connectionId: connectionId };
                    req.body = connectBody;
                    return [4 /*yield*/, httpHandler.createConnection(req, res)];
                case 8:
                    _a.sent();
                    offerBody = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer" };
                    req.body = offerBody;
                    return [4 /*yield*/, httpHandler.postOffer(req, res)];
                case 9:
                    _a.sent();
                    offer = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer", polite: false };
                    return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 10:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.not.arrayContaining([offer]), datetime: expect.anything() });
                    return [4 /*yield*/, httpHandler.getAll(req2, res)];
                case 11:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.arrayContaining([offer]), datetime: expect.anything() });
                    answerBody = { connectionId: connectionId, sdp: testsdp };
                    req2.body = answerBody;
                    return [4 /*yield*/, httpHandler.postAnswer(req2, res)];
                case 12:
                    _a.sent();
                    i = 0;
                    _a.label = 13;
                case 13:
                    if (!(i < RetriesToForceTimeout + 1)) return [3 /*break*/, 17];
                    return [4 /*yield*/, httpHandler.checkSessionId(req3, res, next)];
                case 14:
                    _a.sent();
                    return [4 /*yield*/, new Promise(function (resolve) { return setTimeout(resolve, 1000); })];
                case 15:
                    _a.sent();
                    _a.label = 16;
                case 16:
                    ++i;
                    return [3 /*break*/, 13];
                case 17: 
                // Get all for session3 to trigger cleaning up sessions that timed out.
                return [4 /*yield*/, httpHandler.getAll(req3, res)];
                case 18:
                    // Get all for session3 to trigger cleaning up sessions that timed out.
                    _a.sent();
                    // Check that we do have session3 still
                    return [4 /*yield*/, httpHandler.checkSessionId(req3, res, next)];
                case 19:
                    // Check that we do have session3 still
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenLastCalledWith(200);
                    // Check that we do have session1 still
                    return [4 /*yield*/, httpHandler.checkSessionId(req, res, next)];
                case 20:
                    // Check that we do have session1 still
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenLastCalledWith(404);
                    // Check that we no longer have session2
                    return [4 /*yield*/, httpHandler.checkSessionId(req2, res, next)];
                case 21:
                    // Check that we no longer have session2
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenLastCalledWith(404);
                    return [4 /*yield*/, httpHandler.deleteSession(req3, res)];
                case 22:
                    _a.sent();
                    return [2 /*return*/];
            }
        });
    }); }, 16000);
});
describe('http signaling test in private mode', function () {
    var sessionId = "abcd1234";
    var sessionId2 = "abcd5678";
    var connectionId = "12345";
    var testsdp = "test sdp";
    var _a = (0, express_1.getMockRes)(), res = _a.res, next = _a.next, mockClear = _a.mockClear;
    var req = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId; }) });
    var req2 = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId2; }) });
    beforeAll(function () {
        httpHandler.reset("private");
    });
    beforeEach(function () {
        mockClear();
        httpHandler.checkSessionId(req, res, next);
        httpHandler.checkSessionId(req2, res, next);
    });
    test('throw check has session', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            httpHandler.checkSessionId(req, res, next);
            expect(res.sendStatus).toHaveBeenCalledWith(404);
            expect(next).not.toHaveBeenCalled();
            return [2 /*return*/];
        });
    }); });
    test('create session', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.createSession(sessionId, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ sessionId: sessionId });
                    return [2 /*return*/];
            }
        });
    }); });
    test('create session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.createSession(sessionId2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ sessionId: sessionId2 });
                    return [2 /*return*/];
            }
        });
    }); });
    test('create connection from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId };
                    req.body = body;
                    return [4 /*yield*/, httpHandler.createConnection(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connectionId: connectionId, polite: false, datetime: expect.anything(), type: "connect" });
                    return [2 /*return*/];
            }
        });
    }); });
    test('create connection from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId };
                    req2.body = body;
                    return [4 /*yield*/, httpHandler.createConnection(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connectionId: connectionId, polite: true, datetime: expect.anything(), type: "connect" });
                    return [2 /*return*/];
            }
        });
    }); });
    test('response status 400 if connecctionId does not set', function () { return __awaiter(void 0, void 0, void 0, function () {
        var req3;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    req3 = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId; }) });
                    return [4 /*yield*/, httpHandler.createConnection(req3, res)];
                case 1:
                    _a.sent();
                    expect(res.status).toHaveBeenCalledWith(400);
                    expect(res.send).toHaveBeenCalledWith({ error: new Error("connectionId is required") });
                    return [2 /*return*/];
            }
        });
    }); });
    test('response status 400 if aleady used connection', function () { return __awaiter(void 0, void 0, void 0, function () {
        var sessionId3, body, req3;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    sessionId3 = "session3";
                    return [4 /*yield*/, httpHandler.createSession(sessionId3, res)];
                case 1:
                    _a.sent();
                    body = { connectionId: connectionId };
                    req3 = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId3; }) });
                    req3.body = body;
                    return [4 /*yield*/, httpHandler.createConnection(req3, res)];
                case 2:
                    _a.sent();
                    expect(res.status).toHaveBeenCalledWith(400);
                    expect(res.send).toHaveBeenCalledWith({ error: new Error("".concat(connectionId, ": This connection id is already used.")) });
                    return [2 /*return*/];
            }
        });
    }); });
    test('not connection get from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getConnection(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connections: [{ connectionId: connectionId, datetime: expect.anything(), type: "connect" }] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('post offer from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer" };
                    req.body = body;
                    return [4 /*yield*/, httpHandler.postOffer(req, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('get offer from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getOffer(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ offers: [] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('get offer from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getOffer(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ offers: [{ connectionId: connectionId, sdp: testsdp, polite: true, datetime: expect.anything(), type: "offer" }] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('post answer from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId, sdp: testsdp };
                    req2.body = body;
                    return [4 /*yield*/, httpHandler.postAnswer(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('get answer from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getAnswer(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ answers: [{ connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "answer" }] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('get answer from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getAnswer(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ answers: [] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('post candidate from sesson1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId, candidate: "testcandidate", sdpMLineIndex: 0, sdpMid: 0 };
                    req.body = body;
                    return [4 /*yield*/, httpHandler.postCandidate(req, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('get candidate from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getCandidate(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ candidates: [] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('get candidate from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getCandidate(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ candidates: [{ connectionId: connectionId, candidate: "testcandidate", sdpMLineIndex: 0, sdpMid: 0, type: "candidate", datetime: expect.anything() }] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('delete connection from session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId };
                    req2.body = body;
                    return [4 /*yield*/, httpHandler.deleteConnection(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connectionId: connectionId });
                    return [2 /*return*/];
            }
        });
    }); });
    test('get connection from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0: return [4 /*yield*/, httpHandler.getConnection(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connections: [] });
                    return [2 /*return*/];
            }
        });
    }); });
    test('delete connection from session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var body;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    body = { connectionId: connectionId };
                    req.body = body;
                    return [4 /*yield*/, httpHandler.deleteConnection(req, res)];
                case 1:
                    _a.sent();
                    expect(res.json).toHaveBeenCalledWith({ connectionId: connectionId });
                    return [2 /*return*/];
            }
        });
    }); });
    test('delete session1', function () { return __awaiter(void 0, void 0, void 0, function () {
        var req;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    req = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId; }) });
                    return [4 /*yield*/, httpHandler.deleteSession(req, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('delete session2', function () { return __awaiter(void 0, void 0, void 0, function () {
        var req2;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    req2 = (0, express_1.getMockReq)({ header: jest.fn(function () { return sessionId2; }) });
                    return [4 /*yield*/, httpHandler.deleteSession(req2, res)];
                case 1:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenCalledWith(200);
                    return [2 /*return*/];
            }
        });
    }); });
    test('disconnection get when session2 disconnects before session1 answer', function () { return __awaiter(void 0, void 0, void 0, function () {
        var connectBody, offerBody, offer, deleteBody, answerBody, disconnect;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    httpHandler.reset("private");
                    return [4 /*yield*/, httpHandler.createSession(sessionId, res)];
                case 1:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.createSession(sessionId2, res)];
                case 2:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 3:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: [], datetime: expect.anything() });
                    connectBody = { connectionId: connectionId };
                    req.body = connectBody;
                    return [4 /*yield*/, httpHandler.createConnection(req, res)];
                case 4:
                    _a.sent();
                    req2.body = connectBody;
                    return [4 /*yield*/, httpHandler.createConnection(req2, res)];
                case 5:
                    _a.sent();
                    offerBody = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer" };
                    req.body = offerBody;
                    return [4 /*yield*/, httpHandler.postOffer(req, res)];
                case 6:
                    _a.sent();
                    offer = { connectionId: connectionId, sdp: testsdp, datetime: expect.anything(), type: "offer", polite: true };
                    return [4 /*yield*/, httpHandler.getAll(req, res)];
                case 7:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.not.arrayContaining([offer]), datetime: expect.anything() });
                    return [4 /*yield*/, httpHandler.getAll(req2, res)];
                case 8:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.arrayContaining([offer]), datetime: expect.anything() });
                    deleteBody = { connectionId: connectionId };
                    req2.body = deleteBody;
                    return [4 /*yield*/, httpHandler.deleteConnection(req, res)];
                case 9:
                    _a.sent();
                    return [4 /*yield*/, httpHandler.deleteSession(req, res)];
                case 10:
                    _a.sent();
                    expect(res.sendStatus).toHaveBeenLastCalledWith(200);
                    answerBody = { connectionId: connectionId, sdp: testsdp };
                    req2.body = answerBody;
                    return [4 /*yield*/, httpHandler.postAnswer(req2, res)];
                case 11:
                    _a.sent();
                    disconnect = { connectionId: connectionId, type: "disconnect", datetime: expect.anything() };
                    return [4 /*yield*/, httpHandler.getAll(req2, res)];
                case 12:
                    _a.sent();
                    expect(res.json).toHaveBeenLastCalledWith({ messages: expect.arrayContaining([disconnect]), datetime: expect.anything() });
                    return [4 /*yield*/, httpHandler.deleteSession(req2, res)];
                case 13:
                    _a.sent();
                    return [2 /*return*/];
            }
        });
    }); });
});
//# sourceMappingURL=httphandler.test.js.map