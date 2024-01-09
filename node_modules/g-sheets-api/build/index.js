"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
var gsheetsprocessor_js_1 = __importDefault(require("./gsheetsprocessor.js"));
var reader = function (options, callback, onError) {
    return (0, gsheetsprocessor_js_1.default)(options, function (results) {
        callback(results);
    }, function (error) {
        if (onError) {
            onError(error);
        }
        else {
            throw new Error("g-sheets-api error: ".concat(error));
        }
    });
};
module.exports = reader;
exports.default = reader;
