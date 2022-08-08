/*eslint-disable block-scoped-var, id-length, no-control-regex, no-magic-numbers, no-prototype-builtins, no-redeclare, no-shadow, no-var, sort-vars*/
"use strict";

var $protobuf = require("protobufjs/minimal");

// Common aliases
var $Reader = $protobuf.Reader, $Writer = $protobuf.Writer, $util = $protobuf.util;

// Exported root namespace
var $root = $protobuf.roots["default"] || ($protobuf.roots["default"] = {});

$root.std_msgs = (function() {

    /**
     * Namespace std_msgs.
     * @exports std_msgs
     * @namespace
     */
    var std_msgs = {};

    std_msgs.msg = (function() {

        /**
         * Namespace msg.
         * @memberof std_msgs
         * @namespace
         */
        var msg = {};

        msg.UInt32 = (function() {

            /**
             * Properties of a UInt32.
             * @memberof std_msgs.msg
             * @interface IUInt32
             * @property {number|null} [data] UInt32 data
             */

            /**
             * Constructs a new UInt32.
             * @memberof std_msgs.msg
             * @classdesc Represents a UInt32.
             * @implements IUInt32
             * @constructor
             * @param {std_msgs.msg.IUInt32=} [properties] Properties to set
             */
            function UInt32(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null)
                            this[keys[i]] = properties[keys[i]];
            }

            /**
             * UInt32 data.
             * @member {number} data
             * @memberof std_msgs.msg.UInt32
             * @instance
             */
            UInt32.prototype.data = 0;

            /**
             * Creates a new UInt32 instance using the specified properties.
             * @function create
             * @memberof std_msgs.msg.UInt32
             * @static
             * @param {std_msgs.msg.IUInt32=} [properties] Properties to set
             * @returns {std_msgs.msg.UInt32} UInt32 instance
             */
            UInt32.create = function create(properties) {
                return new UInt32(properties);
            };

            /**
             * Encodes the specified UInt32 message. Does not implicitly {@link std_msgs.msg.UInt32.verify|verify} messages.
             * @function encode
             * @memberof std_msgs.msg.UInt32
             * @static
             * @param {std_msgs.msg.IUInt32} message UInt32 message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            UInt32.encode = function encode(message, writer) {
                if (!writer)
                    writer = $Writer.create();
                if (message.data != null && Object.hasOwnProperty.call(message, "data"))
                    writer.uint32(/* id 2, wireType 0 =*/16).uint32(message.data);
                return writer;
            };

            /**
             * Encodes the specified UInt32 message, length delimited. Does not implicitly {@link std_msgs.msg.UInt32.verify|verify} messages.
             * @function encodeDelimited
             * @memberof std_msgs.msg.UInt32
             * @static
             * @param {std_msgs.msg.IUInt32} message UInt32 message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            UInt32.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a UInt32 message from the specified reader or buffer.
             * @function decode
             * @memberof std_msgs.msg.UInt32
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {std_msgs.msg.UInt32} UInt32
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            UInt32.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader))
                    reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length, message = new $root.std_msgs.msg.UInt32();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                    case 2: {
                            message.data = reader.uint32();
                            break;
                        }
                    default:
                        reader.skipType(tag & 7);
                        break;
                    }
                }
                return message;
            };

            /**
             * Decodes a UInt32 message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof std_msgs.msg.UInt32
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {std_msgs.msg.UInt32} UInt32
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            UInt32.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader))
                    reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a UInt32 message.
             * @function verify
             * @memberof std_msgs.msg.UInt32
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            UInt32.verify = function verify(message) {
                if (typeof message !== "object" || message === null)
                    return "object expected";
                if (message.data != null && message.hasOwnProperty("data"))
                    if (!$util.isInteger(message.data))
                        return "data: integer expected";
                return null;
            };

            /**
             * Creates a UInt32 message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof std_msgs.msg.UInt32
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {std_msgs.msg.UInt32} UInt32
             */
            UInt32.fromObject = function fromObject(object) {
                if (object instanceof $root.std_msgs.msg.UInt32)
                    return object;
                var message = new $root.std_msgs.msg.UInt32();
                if (object.data != null)
                    message.data = object.data >>> 0;
                return message;
            };

            /**
             * Creates a plain object from a UInt32 message. Also converts values to other types if specified.
             * @function toObject
             * @memberof std_msgs.msg.UInt32
             * @static
             * @param {std_msgs.msg.UInt32} message UInt32
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            UInt32.toObject = function toObject(message, options) {
                if (!options)
                    options = {};
                var object = {};
                if (options.defaults)
                    object.data = 0;
                if (message.data != null && message.hasOwnProperty("data"))
                    object.data = message.data;
                return object;
            };

            /**
             * Converts this UInt32 to JSON.
             * @function toJSON
             * @memberof std_msgs.msg.UInt32
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            UInt32.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for UInt32
             * @function getTypeUrl
             * @memberof std_msgs.msg.UInt32
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            UInt32.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = "type.googleapis.com";
                }
                return typeUrlPrefix + "/std_msgs.msg.UInt32";
            };

            return UInt32;
        })();

        return msg;
    })();

    return std_msgs;
})();

module.exports = $root;
