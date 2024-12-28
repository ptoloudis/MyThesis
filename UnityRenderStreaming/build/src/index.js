"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.RenderStreaming = void 0;
var commander_1 = require("commander");
var https = require("https");
var fs = require("fs");
var os = require("os");
var server_1 = require("./server");
var websocket_1 = require("./websocket");
var RenderStreaming = /** @class */ (function () {
    function RenderStreaming(options) {
        var _this = this;
        this.options = options;
        this.app = (0, server_1.createServer)(this.options);
        if (this.options.secure) {
            this.server = https.createServer({
                key: fs.readFileSync(options.keyfile),
                cert: fs.readFileSync(options.certfile),
            }, this.app).listen(this.options.port, function () {
                var port = _this.server.address().port;
                var addresses = _this.getIPAddress();
                for (var _i = 0, addresses_1 = addresses; _i < addresses_1.length; _i++) {
                    var address = addresses_1[_i];
                    console.log("https://".concat(address, ":").concat(port));
                }
            });
        }
        else {
            this.server = this.app.listen(this.options.port, function () {
                var port = _this.server.address().port;
                var addresses = _this.getIPAddress();
                for (var _i = 0, addresses_2 = addresses; _i < addresses_2.length; _i++) {
                    var address = addresses_2[_i];
                    console.log("http://".concat(address, ":").concat(port));
                }
            });
        }
        if (this.options.type == 'http') {
            console.log("Use http polling for signaling server.");
        }
        else if (this.options.type != 'websocket') {
            console.log("signaling type should be set \"websocket\" or \"http\". ".concat(this.options.type, " is not supported."));
            console.log("Changing signaling type to websocket.");
            this.options.type = 'websocket';
        }
        if (this.options.type == 'websocket') {
            console.log("Use websocket for signaling server ws://".concat(this.getIPAddress()[0]));
            //Start Websocket Signaling server
            new websocket_1.default(this.server, this.options.mode);
        }
        console.log("start as ".concat(this.options.mode, " mode"));
    }
    RenderStreaming.run = function (argv) {
        var program = new commander_1.Command();
        var readOptions = function () {
            if (Array.isArray(argv)) {
                program
                    .usage('[options] <apps...>')
                    .option('-p, --port <n>', 'Port to start the server on.', process.env.PORT || "80")
                    .option('-s, --secure', 'Enable HTTPS (you need server.key and server.cert).', process.env.SECURE || false)
                    .option('-k, --keyfile <path>', 'https key file.', process.env.KEYFILE || 'server.key')
                    .option('-c, --certfile <path>', 'https cert file.', process.env.CERTFILE || 'server.cert')
                    .option('-t, --type <type>', 'Type of signaling protocol, Choose websocket or http.', process.env.TYPE || 'websocket')
                    .option('-m, --mode <type>', 'Choose Communication mode public or private.', process.env.MODE || 'public')
                    .option('-l, --logging <type>', 'Choose http logging type combined, dev, short, tiny or none.', process.env.LOGGING || 'dev')
                    .parse(argv);
                var option = program.opts();
                return {
                    port: option.port,
                    secure: option.secure == undefined ? false : option.secure,
                    keyfile: option.keyfile,
                    certfile: option.certfile,
                    type: option.type == undefined ? 'websocket' : option.type,
                    mode: option.mode,
                    logging: option.logging,
                };
            }
        };
        var options = readOptions();
        return new RenderStreaming(options);
    };
    RenderStreaming.prototype.getIPAddress = function () {
        var interfaces = os.networkInterfaces();
        var addresses = [];
        for (var k in interfaces) {
            for (var k2 in interfaces[k]) {
                var address = interfaces[k][k2];
                if (address.family === 'IPv4') {
                    addresses.push(address.address);
                }
            }
        }
        return addresses;
    };
    return RenderStreaming;
}());
exports.RenderStreaming = RenderStreaming;
RenderStreaming.run(process.argv);
//# sourceMappingURL=index.js.map