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

        msg.Header = (function() {

            /**
             * Properties of a Header.
             * @memberof std_msgs.msg
             * @interface IHeader
             * @property {std_msgs.msg.ITime|null} [stamp] Header stamp
             * @property {string|null} [frame_id] Header frame_id
             */

            /**
             * Constructs a new Header.
             * @memberof std_msgs.msg
             * @classdesc Represents a Header.
             * @implements IHeader
             * @constructor
             * @param {std_msgs.msg.IHeader=} [properties] Properties to set
             */
            function Header(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null)
                            this[keys[i]] = properties[keys[i]];
            }

            /**
             * Header stamp.
             * @member {std_msgs.msg.ITime|null|undefined} stamp
             * @memberof std_msgs.msg.Header
             * @instance
             */
            Header.prototype.stamp = null;

            /**
             * Header frame_id.
             * @member {string} frame_id
             * @memberof std_msgs.msg.Header
             * @instance
             */
            Header.prototype.frame_id = "";

            /**
             * Creates a new Header instance using the specified properties.
             * @function create
             * @memberof std_msgs.msg.Header
             * @static
             * @param {std_msgs.msg.IHeader=} [properties] Properties to set
             * @returns {std_msgs.msg.Header} Header instance
             */
            Header.create = function create(properties) {
                return new Header(properties);
            };

            /**
             * Encodes the specified Header message. Does not implicitly {@link std_msgs.msg.Header.verify|verify} messages.
             * @function encode
             * @memberof std_msgs.msg.Header
             * @static
             * @param {std_msgs.msg.IHeader} message Header message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Header.encode = function encode(message, writer) {
                if (!writer)
                    writer = $Writer.create();
                if (message.stamp != null && Object.hasOwnProperty.call(message, "stamp"))
                    $root.std_msgs.msg.Time.encode(message.stamp, writer.uint32(/* id 1, wireType 2 =*/10).fork()).ldelim();
                if (message.frame_id != null && Object.hasOwnProperty.call(message, "frame_id"))
                    writer.uint32(/* id 2, wireType 2 =*/18).string(message.frame_id);
                return writer;
            };

            /**
             * Encodes the specified Header message, length delimited. Does not implicitly {@link std_msgs.msg.Header.verify|verify} messages.
             * @function encodeDelimited
             * @memberof std_msgs.msg.Header
             * @static
             * @param {std_msgs.msg.IHeader} message Header message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Header.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a Header message from the specified reader or buffer.
             * @function decode
             * @memberof std_msgs.msg.Header
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {std_msgs.msg.Header} Header
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Header.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader))
                    reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length, message = new $root.std_msgs.msg.Header();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                    case 1: {
                            message.stamp = $root.std_msgs.msg.Time.decode(reader, reader.uint32());
                            break;
                        }
                    case 2: {
                            message.frame_id = reader.string();
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
             * Decodes a Header message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof std_msgs.msg.Header
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {std_msgs.msg.Header} Header
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Header.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader))
                    reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a Header message.
             * @function verify
             * @memberof std_msgs.msg.Header
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            Header.verify = function verify(message) {
                if (typeof message !== "object" || message === null)
                    return "object expected";
                if (message.stamp != null && message.hasOwnProperty("stamp")) {
                    var error = $root.std_msgs.msg.Time.verify(message.stamp);
                    if (error)
                        return "stamp." + error;
                }
                if (message.frame_id != null && message.hasOwnProperty("frame_id"))
                    if (!$util.isString(message.frame_id))
                        return "frame_id: string expected";
                return null;
            };

            /**
             * Creates a Header message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof std_msgs.msg.Header
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {std_msgs.msg.Header} Header
             */
            Header.fromObject = function fromObject(object) {
                if (object instanceof $root.std_msgs.msg.Header)
                    return object;
                var message = new $root.std_msgs.msg.Header();
                if (object.stamp != null) {
                    if (typeof object.stamp !== "object")
                        throw TypeError(".std_msgs.msg.Header.stamp: object expected");
                    message.stamp = $root.std_msgs.msg.Time.fromObject(object.stamp);
                }
                if (object.frame_id != null)
                    message.frame_id = String(object.frame_id);
                return message;
            };

            /**
             * Creates a plain object from a Header message. Also converts values to other types if specified.
             * @function toObject
             * @memberof std_msgs.msg.Header
             * @static
             * @param {std_msgs.msg.Header} message Header
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            Header.toObject = function toObject(message, options) {
                if (!options)
                    options = {};
                var object = {};
                if (options.defaults) {
                    object.stamp = null;
                    object.frame_id = "";
                }
                if (message.stamp != null && message.hasOwnProperty("stamp"))
                    object.stamp = $root.std_msgs.msg.Time.toObject(message.stamp, options);
                if (message.frame_id != null && message.hasOwnProperty("frame_id"))
                    object.frame_id = message.frame_id;
                return object;
            };

            /**
             * Converts this Header to JSON.
             * @function toJSON
             * @memberof std_msgs.msg.Header
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            Header.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for Header
             * @function getTypeUrl
             * @memberof std_msgs.msg.Header
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            Header.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = "type.googleapis.com";
                }
                return typeUrlPrefix + "/std_msgs.msg.Header";
            };

            return Header;
        })();

        msg.Time = (function() {

            /**
             * Properties of a Time.
             * @memberof std_msgs.msg
             * @interface ITime
             * @property {number|null} [sec] Time sec
             * @property {number|null} [nanosec] Time nanosec
             */

            /**
             * Constructs a new Time.
             * @memberof std_msgs.msg
             * @classdesc Represents a Time.
             * @implements ITime
             * @constructor
             * @param {std_msgs.msg.ITime=} [properties] Properties to set
             */
            function Time(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null)
                            this[keys[i]] = properties[keys[i]];
            }

            /**
             * Time sec.
             * @member {number} sec
             * @memberof std_msgs.msg.Time
             * @instance
             */
            Time.prototype.sec = 0;

            /**
             * Time nanosec.
             * @member {number} nanosec
             * @memberof std_msgs.msg.Time
             * @instance
             */
            Time.prototype.nanosec = 0;

            /**
             * Creates a new Time instance using the specified properties.
             * @function create
             * @memberof std_msgs.msg.Time
             * @static
             * @param {std_msgs.msg.ITime=} [properties] Properties to set
             * @returns {std_msgs.msg.Time} Time instance
             */
            Time.create = function create(properties) {
                return new Time(properties);
            };

            /**
             * Encodes the specified Time message. Does not implicitly {@link std_msgs.msg.Time.verify|verify} messages.
             * @function encode
             * @memberof std_msgs.msg.Time
             * @static
             * @param {std_msgs.msg.ITime} message Time message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Time.encode = function encode(message, writer) {
                if (!writer)
                    writer = $Writer.create();
                if (message.sec != null && Object.hasOwnProperty.call(message, "sec"))
                    writer.uint32(/* id 1, wireType 0 =*/8).int32(message.sec);
                if (message.nanosec != null && Object.hasOwnProperty.call(message, "nanosec"))
                    writer.uint32(/* id 2, wireType 0 =*/16).uint32(message.nanosec);
                return writer;
            };

            /**
             * Encodes the specified Time message, length delimited. Does not implicitly {@link std_msgs.msg.Time.verify|verify} messages.
             * @function encodeDelimited
             * @memberof std_msgs.msg.Time
             * @static
             * @param {std_msgs.msg.ITime} message Time message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Time.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a Time message from the specified reader or buffer.
             * @function decode
             * @memberof std_msgs.msg.Time
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {std_msgs.msg.Time} Time
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Time.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader))
                    reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length, message = new $root.std_msgs.msg.Time();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                    case 1: {
                            message.sec = reader.int32();
                            break;
                        }
                    case 2: {
                            message.nanosec = reader.uint32();
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
             * Decodes a Time message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof std_msgs.msg.Time
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {std_msgs.msg.Time} Time
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Time.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader))
                    reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a Time message.
             * @function verify
             * @memberof std_msgs.msg.Time
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            Time.verify = function verify(message) {
                if (typeof message !== "object" || message === null)
                    return "object expected";
                if (message.sec != null && message.hasOwnProperty("sec"))
                    if (!$util.isInteger(message.sec))
                        return "sec: integer expected";
                if (message.nanosec != null && message.hasOwnProperty("nanosec"))
                    if (!$util.isInteger(message.nanosec))
                        return "nanosec: integer expected";
                return null;
            };

            /**
             * Creates a Time message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof std_msgs.msg.Time
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {std_msgs.msg.Time} Time
             */
            Time.fromObject = function fromObject(object) {
                if (object instanceof $root.std_msgs.msg.Time)
                    return object;
                var message = new $root.std_msgs.msg.Time();
                if (object.sec != null)
                    message.sec = object.sec | 0;
                if (object.nanosec != null)
                    message.nanosec = object.nanosec >>> 0;
                return message;
            };

            /**
             * Creates a plain object from a Time message. Also converts values to other types if specified.
             * @function toObject
             * @memberof std_msgs.msg.Time
             * @static
             * @param {std_msgs.msg.Time} message Time
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            Time.toObject = function toObject(message, options) {
                if (!options)
                    options = {};
                var object = {};
                if (options.defaults) {
                    object.sec = 0;
                    object.nanosec = 0;
                }
                if (message.sec != null && message.hasOwnProperty("sec"))
                    object.sec = message.sec;
                if (message.nanosec != null && message.hasOwnProperty("nanosec"))
                    object.nanosec = message.nanosec;
                return object;
            };

            /**
             * Converts this Time to JSON.
             * @function toJSON
             * @memberof std_msgs.msg.Time
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            Time.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for Time
             * @function getTypeUrl
             * @memberof std_msgs.msg.Time
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            Time.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = "type.googleapis.com";
                }
                return typeUrlPrefix + "/std_msgs.msg.Time";
            };

            return Time;
        })();

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

        msg.String = (function() {

            /**
             * Properties of a String.
             * @memberof std_msgs.msg
             * @interface IString
             * @property {string|null} [data] String data
             */

            /**
             * Constructs a new String.
             * @memberof std_msgs.msg
             * @classdesc Represents a String.
             * @implements IString
             * @constructor
             * @param {std_msgs.msg.IString=} [properties] Properties to set
             */
            function String(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null)
                            this[keys[i]] = properties[keys[i]];
            }

            /**
             * String data.
             * @member {string} data
             * @memberof std_msgs.msg.String
             * @instance
             */
            String.prototype.data = "";

            /**
             * Creates a new String instance using the specified properties.
             * @function create
             * @memberof std_msgs.msg.String
             * @static
             * @param {std_msgs.msg.IString=} [properties] Properties to set
             * @returns {std_msgs.msg.String} String instance
             */
            String.create = function create(properties) {
                return new String(properties);
            };

            /**
             * Encodes the specified String message. Does not implicitly {@link std_msgs.msg.String.verify|verify} messages.
             * @function encode
             * @memberof std_msgs.msg.String
             * @static
             * @param {std_msgs.msg.IString} message String message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            String.encode = function encode(message, writer) {
                if (!writer)
                    writer = $Writer.create();
                if (message.data != null && Object.hasOwnProperty.call(message, "data"))
                    writer.uint32(/* id 1, wireType 2 =*/10).string(message.data);
                return writer;
            };

            /**
             * Encodes the specified String message, length delimited. Does not implicitly {@link std_msgs.msg.String.verify|verify} messages.
             * @function encodeDelimited
             * @memberof std_msgs.msg.String
             * @static
             * @param {std_msgs.msg.IString} message String message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            String.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a String message from the specified reader or buffer.
             * @function decode
             * @memberof std_msgs.msg.String
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {std_msgs.msg.String} String
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            String.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader))
                    reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length, message = new $root.std_msgs.msg.String();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                    case 1: {
                            message.data = reader.string();
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
             * Decodes a String message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof std_msgs.msg.String
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {std_msgs.msg.String} String
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            String.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader))
                    reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a String message.
             * @function verify
             * @memberof std_msgs.msg.String
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            String.verify = function verify(message) {
                if (typeof message !== "object" || message === null)
                    return "object expected";
                if (message.data != null && message.hasOwnProperty("data"))
                    if (!$util.isString(message.data))
                        return "data: string expected";
                return null;
            };

            /**
             * Creates a String message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof std_msgs.msg.String
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {std_msgs.msg.String} String
             */
            String.fromObject = function fromObject(object) {
                if (object instanceof $root.std_msgs.msg.String)
                    return object;
                var message = new $root.std_msgs.msg.String();
                if (object.data != null)
                    message.data = String(object.data);
                return message;
            };

            /**
             * Creates a plain object from a String message. Also converts values to other types if specified.
             * @function toObject
             * @memberof std_msgs.msg.String
             * @static
             * @param {std_msgs.msg.String} message String
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            String.toObject = function toObject(message, options) {
                if (!options)
                    options = {};
                var object = {};
                if (options.defaults)
                    object.data = "";
                if (message.data != null && message.hasOwnProperty("data"))
                    object.data = message.data;
                return object;
            };

            /**
             * Converts this String to JSON.
             * @function toJSON
             * @memberof std_msgs.msg.String
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            String.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for String
             * @function getTypeUrl
             * @memberof std_msgs.msg.String
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            String.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = "type.googleapis.com";
                }
                return typeUrlPrefix + "/std_msgs.msg.String";
            };

            return String;
        })();

        return msg;
    })();

    return std_msgs;
})();

$root.geometry_msgs = (function() {

    /**
     * Namespace geometry_msgs.
     * @exports geometry_msgs
     * @namespace
     */
    var geometry_msgs = {};

    geometry_msgs.msg = (function() {

        /**
         * Namespace msg.
         * @memberof geometry_msgs
         * @namespace
         */
        var msg = {};

        msg.PoseStamped = (function() {

            /**
             * Properties of a PoseStamped.
             * @memberof geometry_msgs.msg
             * @interface IPoseStamped
             * @property {std_msgs.msg.IHeader|null} [header] PoseStamped header
             * @property {geometry_msgs.msg.IPose|null} [pose] PoseStamped pose
             */

            /**
             * Constructs a new PoseStamped.
             * @memberof geometry_msgs.msg
             * @classdesc Represents a PoseStamped.
             * @implements IPoseStamped
             * @constructor
             * @param {geometry_msgs.msg.IPoseStamped=} [properties] Properties to set
             */
            function PoseStamped(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null)
                            this[keys[i]] = properties[keys[i]];
            }

            /**
             * PoseStamped header.
             * @member {std_msgs.msg.IHeader|null|undefined} header
             * @memberof geometry_msgs.msg.PoseStamped
             * @instance
             */
            PoseStamped.prototype.header = null;

            /**
             * PoseStamped pose.
             * @member {geometry_msgs.msg.IPose|null|undefined} pose
             * @memberof geometry_msgs.msg.PoseStamped
             * @instance
             */
            PoseStamped.prototype.pose = null;

            /**
             * Creates a new PoseStamped instance using the specified properties.
             * @function create
             * @memberof geometry_msgs.msg.PoseStamped
             * @static
             * @param {geometry_msgs.msg.IPoseStamped=} [properties] Properties to set
             * @returns {geometry_msgs.msg.PoseStamped} PoseStamped instance
             */
            PoseStamped.create = function create(properties) {
                return new PoseStamped(properties);
            };

            /**
             * Encodes the specified PoseStamped message. Does not implicitly {@link geometry_msgs.msg.PoseStamped.verify|verify} messages.
             * @function encode
             * @memberof geometry_msgs.msg.PoseStamped
             * @static
             * @param {geometry_msgs.msg.IPoseStamped} message PoseStamped message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            PoseStamped.encode = function encode(message, writer) {
                if (!writer)
                    writer = $Writer.create();
                if (message.header != null && Object.hasOwnProperty.call(message, "header"))
                    $root.std_msgs.msg.Header.encode(message.header, writer.uint32(/* id 1, wireType 2 =*/10).fork()).ldelim();
                if (message.pose != null && Object.hasOwnProperty.call(message, "pose"))
                    $root.geometry_msgs.msg.Pose.encode(message.pose, writer.uint32(/* id 2, wireType 2 =*/18).fork()).ldelim();
                return writer;
            };

            /**
             * Encodes the specified PoseStamped message, length delimited. Does not implicitly {@link geometry_msgs.msg.PoseStamped.verify|verify} messages.
             * @function encodeDelimited
             * @memberof geometry_msgs.msg.PoseStamped
             * @static
             * @param {geometry_msgs.msg.IPoseStamped} message PoseStamped message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            PoseStamped.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a PoseStamped message from the specified reader or buffer.
             * @function decode
             * @memberof geometry_msgs.msg.PoseStamped
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {geometry_msgs.msg.PoseStamped} PoseStamped
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            PoseStamped.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader))
                    reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length, message = new $root.geometry_msgs.msg.PoseStamped();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                    case 1: {
                            message.header = $root.std_msgs.msg.Header.decode(reader, reader.uint32());
                            break;
                        }
                    case 2: {
                            message.pose = $root.geometry_msgs.msg.Pose.decode(reader, reader.uint32());
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
             * Decodes a PoseStamped message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof geometry_msgs.msg.PoseStamped
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {geometry_msgs.msg.PoseStamped} PoseStamped
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            PoseStamped.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader))
                    reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a PoseStamped message.
             * @function verify
             * @memberof geometry_msgs.msg.PoseStamped
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            PoseStamped.verify = function verify(message) {
                if (typeof message !== "object" || message === null)
                    return "object expected";
                if (message.header != null && message.hasOwnProperty("header")) {
                    var error = $root.std_msgs.msg.Header.verify(message.header);
                    if (error)
                        return "header." + error;
                }
                if (message.pose != null && message.hasOwnProperty("pose")) {
                    var error = $root.geometry_msgs.msg.Pose.verify(message.pose);
                    if (error)
                        return "pose." + error;
                }
                return null;
            };

            /**
             * Creates a PoseStamped message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof geometry_msgs.msg.PoseStamped
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {geometry_msgs.msg.PoseStamped} PoseStamped
             */
            PoseStamped.fromObject = function fromObject(object) {
                if (object instanceof $root.geometry_msgs.msg.PoseStamped)
                    return object;
                var message = new $root.geometry_msgs.msg.PoseStamped();
                if (object.header != null) {
                    if (typeof object.header !== "object")
                        throw TypeError(".geometry_msgs.msg.PoseStamped.header: object expected");
                    message.header = $root.std_msgs.msg.Header.fromObject(object.header);
                }
                if (object.pose != null) {
                    if (typeof object.pose !== "object")
                        throw TypeError(".geometry_msgs.msg.PoseStamped.pose: object expected");
                    message.pose = $root.geometry_msgs.msg.Pose.fromObject(object.pose);
                }
                return message;
            };

            /**
             * Creates a plain object from a PoseStamped message. Also converts values to other types if specified.
             * @function toObject
             * @memberof geometry_msgs.msg.PoseStamped
             * @static
             * @param {geometry_msgs.msg.PoseStamped} message PoseStamped
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            PoseStamped.toObject = function toObject(message, options) {
                if (!options)
                    options = {};
                var object = {};
                if (options.defaults) {
                    object.header = null;
                    object.pose = null;
                }
                if (message.header != null && message.hasOwnProperty("header"))
                    object.header = $root.std_msgs.msg.Header.toObject(message.header, options);
                if (message.pose != null && message.hasOwnProperty("pose"))
                    object.pose = $root.geometry_msgs.msg.Pose.toObject(message.pose, options);
                return object;
            };

            /**
             * Converts this PoseStamped to JSON.
             * @function toJSON
             * @memberof geometry_msgs.msg.PoseStamped
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            PoseStamped.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for PoseStamped
             * @function getTypeUrl
             * @memberof geometry_msgs.msg.PoseStamped
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            PoseStamped.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = "type.googleapis.com";
                }
                return typeUrlPrefix + "/geometry_msgs.msg.PoseStamped";
            };

            return PoseStamped;
        })();

        msg.Pose = (function() {

            /**
             * Properties of a Pose.
             * @memberof geometry_msgs.msg
             * @interface IPose
             * @property {geometry_msgs.msg.IPoint|null} [position] Pose position
             * @property {geometry_msgs.msg.IQuaternion|null} [orientation] Pose orientation
             */

            /**
             * Constructs a new Pose.
             * @memberof geometry_msgs.msg
             * @classdesc Represents a Pose.
             * @implements IPose
             * @constructor
             * @param {geometry_msgs.msg.IPose=} [properties] Properties to set
             */
            function Pose(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null)
                            this[keys[i]] = properties[keys[i]];
            }

            /**
             * Pose position.
             * @member {geometry_msgs.msg.IPoint|null|undefined} position
             * @memberof geometry_msgs.msg.Pose
             * @instance
             */
            Pose.prototype.position = null;

            /**
             * Pose orientation.
             * @member {geometry_msgs.msg.IQuaternion|null|undefined} orientation
             * @memberof geometry_msgs.msg.Pose
             * @instance
             */
            Pose.prototype.orientation = null;

            /**
             * Creates a new Pose instance using the specified properties.
             * @function create
             * @memberof geometry_msgs.msg.Pose
             * @static
             * @param {geometry_msgs.msg.IPose=} [properties] Properties to set
             * @returns {geometry_msgs.msg.Pose} Pose instance
             */
            Pose.create = function create(properties) {
                return new Pose(properties);
            };

            /**
             * Encodes the specified Pose message. Does not implicitly {@link geometry_msgs.msg.Pose.verify|verify} messages.
             * @function encode
             * @memberof geometry_msgs.msg.Pose
             * @static
             * @param {geometry_msgs.msg.IPose} message Pose message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Pose.encode = function encode(message, writer) {
                if (!writer)
                    writer = $Writer.create();
                if (message.position != null && Object.hasOwnProperty.call(message, "position"))
                    $root.geometry_msgs.msg.Point.encode(message.position, writer.uint32(/* id 1, wireType 2 =*/10).fork()).ldelim();
                if (message.orientation != null && Object.hasOwnProperty.call(message, "orientation"))
                    $root.geometry_msgs.msg.Quaternion.encode(message.orientation, writer.uint32(/* id 2, wireType 2 =*/18).fork()).ldelim();
                return writer;
            };

            /**
             * Encodes the specified Pose message, length delimited. Does not implicitly {@link geometry_msgs.msg.Pose.verify|verify} messages.
             * @function encodeDelimited
             * @memberof geometry_msgs.msg.Pose
             * @static
             * @param {geometry_msgs.msg.IPose} message Pose message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Pose.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a Pose message from the specified reader or buffer.
             * @function decode
             * @memberof geometry_msgs.msg.Pose
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {geometry_msgs.msg.Pose} Pose
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Pose.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader))
                    reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length, message = new $root.geometry_msgs.msg.Pose();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                    case 1: {
                            message.position = $root.geometry_msgs.msg.Point.decode(reader, reader.uint32());
                            break;
                        }
                    case 2: {
                            message.orientation = $root.geometry_msgs.msg.Quaternion.decode(reader, reader.uint32());
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
             * Decodes a Pose message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof geometry_msgs.msg.Pose
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {geometry_msgs.msg.Pose} Pose
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Pose.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader))
                    reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a Pose message.
             * @function verify
             * @memberof geometry_msgs.msg.Pose
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            Pose.verify = function verify(message) {
                if (typeof message !== "object" || message === null)
                    return "object expected";
                if (message.position != null && message.hasOwnProperty("position")) {
                    var error = $root.geometry_msgs.msg.Point.verify(message.position);
                    if (error)
                        return "position." + error;
                }
                if (message.orientation != null && message.hasOwnProperty("orientation")) {
                    var error = $root.geometry_msgs.msg.Quaternion.verify(message.orientation);
                    if (error)
                        return "orientation." + error;
                }
                return null;
            };

            /**
             * Creates a Pose message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof geometry_msgs.msg.Pose
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {geometry_msgs.msg.Pose} Pose
             */
            Pose.fromObject = function fromObject(object) {
                if (object instanceof $root.geometry_msgs.msg.Pose)
                    return object;
                var message = new $root.geometry_msgs.msg.Pose();
                if (object.position != null) {
                    if (typeof object.position !== "object")
                        throw TypeError(".geometry_msgs.msg.Pose.position: object expected");
                    message.position = $root.geometry_msgs.msg.Point.fromObject(object.position);
                }
                if (object.orientation != null) {
                    if (typeof object.orientation !== "object")
                        throw TypeError(".geometry_msgs.msg.Pose.orientation: object expected");
                    message.orientation = $root.geometry_msgs.msg.Quaternion.fromObject(object.orientation);
                }
                return message;
            };

            /**
             * Creates a plain object from a Pose message. Also converts values to other types if specified.
             * @function toObject
             * @memberof geometry_msgs.msg.Pose
             * @static
             * @param {geometry_msgs.msg.Pose} message Pose
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            Pose.toObject = function toObject(message, options) {
                if (!options)
                    options = {};
                var object = {};
                if (options.defaults) {
                    object.position = null;
                    object.orientation = null;
                }
                if (message.position != null && message.hasOwnProperty("position"))
                    object.position = $root.geometry_msgs.msg.Point.toObject(message.position, options);
                if (message.orientation != null && message.hasOwnProperty("orientation"))
                    object.orientation = $root.geometry_msgs.msg.Quaternion.toObject(message.orientation, options);
                return object;
            };

            /**
             * Converts this Pose to JSON.
             * @function toJSON
             * @memberof geometry_msgs.msg.Pose
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            Pose.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for Pose
             * @function getTypeUrl
             * @memberof geometry_msgs.msg.Pose
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            Pose.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = "type.googleapis.com";
                }
                return typeUrlPrefix + "/geometry_msgs.msg.Pose";
            };

            return Pose;
        })();

        msg.Point = (function() {

            /**
             * Properties of a Point.
             * @memberof geometry_msgs.msg
             * @interface IPoint
             * @property {number|null} [x] Point x
             * @property {number|null} [y] Point y
             * @property {number|null} [z] Point z
             */

            /**
             * Constructs a new Point.
             * @memberof geometry_msgs.msg
             * @classdesc Represents a Point.
             * @implements IPoint
             * @constructor
             * @param {geometry_msgs.msg.IPoint=} [properties] Properties to set
             */
            function Point(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null)
                            this[keys[i]] = properties[keys[i]];
            }

            /**
             * Point x.
             * @member {number} x
             * @memberof geometry_msgs.msg.Point
             * @instance
             */
            Point.prototype.x = 0;

            /**
             * Point y.
             * @member {number} y
             * @memberof geometry_msgs.msg.Point
             * @instance
             */
            Point.prototype.y = 0;

            /**
             * Point z.
             * @member {number} z
             * @memberof geometry_msgs.msg.Point
             * @instance
             */
            Point.prototype.z = 0;

            /**
             * Creates a new Point instance using the specified properties.
             * @function create
             * @memberof geometry_msgs.msg.Point
             * @static
             * @param {geometry_msgs.msg.IPoint=} [properties] Properties to set
             * @returns {geometry_msgs.msg.Point} Point instance
             */
            Point.create = function create(properties) {
                return new Point(properties);
            };

            /**
             * Encodes the specified Point message. Does not implicitly {@link geometry_msgs.msg.Point.verify|verify} messages.
             * @function encode
             * @memberof geometry_msgs.msg.Point
             * @static
             * @param {geometry_msgs.msg.IPoint} message Point message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Point.encode = function encode(message, writer) {
                if (!writer)
                    writer = $Writer.create();
                if (message.x != null && Object.hasOwnProperty.call(message, "x"))
                    writer.uint32(/* id 1, wireType 1 =*/9).double(message.x);
                if (message.y != null && Object.hasOwnProperty.call(message, "y"))
                    writer.uint32(/* id 2, wireType 1 =*/17).double(message.y);
                if (message.z != null && Object.hasOwnProperty.call(message, "z"))
                    writer.uint32(/* id 3, wireType 1 =*/25).double(message.z);
                return writer;
            };

            /**
             * Encodes the specified Point message, length delimited. Does not implicitly {@link geometry_msgs.msg.Point.verify|verify} messages.
             * @function encodeDelimited
             * @memberof geometry_msgs.msg.Point
             * @static
             * @param {geometry_msgs.msg.IPoint} message Point message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Point.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a Point message from the specified reader or buffer.
             * @function decode
             * @memberof geometry_msgs.msg.Point
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {geometry_msgs.msg.Point} Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Point.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader))
                    reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length, message = new $root.geometry_msgs.msg.Point();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                    case 1: {
                            message.x = reader.double();
                            break;
                        }
                    case 2: {
                            message.y = reader.double();
                            break;
                        }
                    case 3: {
                            message.z = reader.double();
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
             * Decodes a Point message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof geometry_msgs.msg.Point
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {geometry_msgs.msg.Point} Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Point.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader))
                    reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a Point message.
             * @function verify
             * @memberof geometry_msgs.msg.Point
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            Point.verify = function verify(message) {
                if (typeof message !== "object" || message === null)
                    return "object expected";
                if (message.x != null && message.hasOwnProperty("x"))
                    if (typeof message.x !== "number")
                        return "x: number expected";
                if (message.y != null && message.hasOwnProperty("y"))
                    if (typeof message.y !== "number")
                        return "y: number expected";
                if (message.z != null && message.hasOwnProperty("z"))
                    if (typeof message.z !== "number")
                        return "z: number expected";
                return null;
            };

            /**
             * Creates a Point message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof geometry_msgs.msg.Point
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {geometry_msgs.msg.Point} Point
             */
            Point.fromObject = function fromObject(object) {
                if (object instanceof $root.geometry_msgs.msg.Point)
                    return object;
                var message = new $root.geometry_msgs.msg.Point();
                if (object.x != null)
                    message.x = Number(object.x);
                if (object.y != null)
                    message.y = Number(object.y);
                if (object.z != null)
                    message.z = Number(object.z);
                return message;
            };

            /**
             * Creates a plain object from a Point message. Also converts values to other types if specified.
             * @function toObject
             * @memberof geometry_msgs.msg.Point
             * @static
             * @param {geometry_msgs.msg.Point} message Point
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            Point.toObject = function toObject(message, options) {
                if (!options)
                    options = {};
                var object = {};
                if (options.defaults) {
                    object.x = 0;
                    object.y = 0;
                    object.z = 0;
                }
                if (message.x != null && message.hasOwnProperty("x"))
                    object.x = options.json && !isFinite(message.x) ? String(message.x) : message.x;
                if (message.y != null && message.hasOwnProperty("y"))
                    object.y = options.json && !isFinite(message.y) ? String(message.y) : message.y;
                if (message.z != null && message.hasOwnProperty("z"))
                    object.z = options.json && !isFinite(message.z) ? String(message.z) : message.z;
                return object;
            };

            /**
             * Converts this Point to JSON.
             * @function toJSON
             * @memberof geometry_msgs.msg.Point
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            Point.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for Point
             * @function getTypeUrl
             * @memberof geometry_msgs.msg.Point
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            Point.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = "type.googleapis.com";
                }
                return typeUrlPrefix + "/geometry_msgs.msg.Point";
            };

            return Point;
        })();

        msg.Quaternion = (function() {

            /**
             * Properties of a Quaternion.
             * @memberof geometry_msgs.msg
             * @interface IQuaternion
             * @property {number|null} [x] Quaternion x
             * @property {number|null} [y] Quaternion y
             * @property {number|null} [z] Quaternion z
             * @property {number|null} [w] Quaternion w
             */

            /**
             * Constructs a new Quaternion.
             * @memberof geometry_msgs.msg
             * @classdesc Represents a Quaternion.
             * @implements IQuaternion
             * @constructor
             * @param {geometry_msgs.msg.IQuaternion=} [properties] Properties to set
             */
            function Quaternion(properties) {
                if (properties)
                    for (var keys = Object.keys(properties), i = 0; i < keys.length; ++i)
                        if (properties[keys[i]] != null)
                            this[keys[i]] = properties[keys[i]];
            }

            /**
             * Quaternion x.
             * @member {number} x
             * @memberof geometry_msgs.msg.Quaternion
             * @instance
             */
            Quaternion.prototype.x = 0;

            /**
             * Quaternion y.
             * @member {number} y
             * @memberof geometry_msgs.msg.Quaternion
             * @instance
             */
            Quaternion.prototype.y = 0;

            /**
             * Quaternion z.
             * @member {number} z
             * @memberof geometry_msgs.msg.Quaternion
             * @instance
             */
            Quaternion.prototype.z = 0;

            /**
             * Quaternion w.
             * @member {number} w
             * @memberof geometry_msgs.msg.Quaternion
             * @instance
             */
            Quaternion.prototype.w = 0;

            /**
             * Creates a new Quaternion instance using the specified properties.
             * @function create
             * @memberof geometry_msgs.msg.Quaternion
             * @static
             * @param {geometry_msgs.msg.IQuaternion=} [properties] Properties to set
             * @returns {geometry_msgs.msg.Quaternion} Quaternion instance
             */
            Quaternion.create = function create(properties) {
                return new Quaternion(properties);
            };

            /**
             * Encodes the specified Quaternion message. Does not implicitly {@link geometry_msgs.msg.Quaternion.verify|verify} messages.
             * @function encode
             * @memberof geometry_msgs.msg.Quaternion
             * @static
             * @param {geometry_msgs.msg.IQuaternion} message Quaternion message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Quaternion.encode = function encode(message, writer) {
                if (!writer)
                    writer = $Writer.create();
                if (message.x != null && Object.hasOwnProperty.call(message, "x"))
                    writer.uint32(/* id 1, wireType 1 =*/9).double(message.x);
                if (message.y != null && Object.hasOwnProperty.call(message, "y"))
                    writer.uint32(/* id 2, wireType 1 =*/17).double(message.y);
                if (message.z != null && Object.hasOwnProperty.call(message, "z"))
                    writer.uint32(/* id 3, wireType 1 =*/25).double(message.z);
                if (message.w != null && Object.hasOwnProperty.call(message, "w"))
                    writer.uint32(/* id 4, wireType 1 =*/33).double(message.w);
                return writer;
            };

            /**
             * Encodes the specified Quaternion message, length delimited. Does not implicitly {@link geometry_msgs.msg.Quaternion.verify|verify} messages.
             * @function encodeDelimited
             * @memberof geometry_msgs.msg.Quaternion
             * @static
             * @param {geometry_msgs.msg.IQuaternion} message Quaternion message or plain object to encode
             * @param {$protobuf.Writer} [writer] Writer to encode to
             * @returns {$protobuf.Writer} Writer
             */
            Quaternion.encodeDelimited = function encodeDelimited(message, writer) {
                return this.encode(message, writer).ldelim();
            };

            /**
             * Decodes a Quaternion message from the specified reader or buffer.
             * @function decode
             * @memberof geometry_msgs.msg.Quaternion
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @param {number} [length] Message length if known beforehand
             * @returns {geometry_msgs.msg.Quaternion} Quaternion
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Quaternion.decode = function decode(reader, length) {
                if (!(reader instanceof $Reader))
                    reader = $Reader.create(reader);
                var end = length === undefined ? reader.len : reader.pos + length, message = new $root.geometry_msgs.msg.Quaternion();
                while (reader.pos < end) {
                    var tag = reader.uint32();
                    switch (tag >>> 3) {
                    case 1: {
                            message.x = reader.double();
                            break;
                        }
                    case 2: {
                            message.y = reader.double();
                            break;
                        }
                    case 3: {
                            message.z = reader.double();
                            break;
                        }
                    case 4: {
                            message.w = reader.double();
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
             * Decodes a Quaternion message from the specified reader or buffer, length delimited.
             * @function decodeDelimited
             * @memberof geometry_msgs.msg.Quaternion
             * @static
             * @param {$protobuf.Reader|Uint8Array} reader Reader or buffer to decode from
             * @returns {geometry_msgs.msg.Quaternion} Quaternion
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            Quaternion.decodeDelimited = function decodeDelimited(reader) {
                if (!(reader instanceof $Reader))
                    reader = new $Reader(reader);
                return this.decode(reader, reader.uint32());
            };

            /**
             * Verifies a Quaternion message.
             * @function verify
             * @memberof geometry_msgs.msg.Quaternion
             * @static
             * @param {Object.<string,*>} message Plain object to verify
             * @returns {string|null} `null` if valid, otherwise the reason why it is not
             */
            Quaternion.verify = function verify(message) {
                if (typeof message !== "object" || message === null)
                    return "object expected";
                if (message.x != null && message.hasOwnProperty("x"))
                    if (typeof message.x !== "number")
                        return "x: number expected";
                if (message.y != null && message.hasOwnProperty("y"))
                    if (typeof message.y !== "number")
                        return "y: number expected";
                if (message.z != null && message.hasOwnProperty("z"))
                    if (typeof message.z !== "number")
                        return "z: number expected";
                if (message.w != null && message.hasOwnProperty("w"))
                    if (typeof message.w !== "number")
                        return "w: number expected";
                return null;
            };

            /**
             * Creates a Quaternion message from a plain object. Also converts values to their respective internal types.
             * @function fromObject
             * @memberof geometry_msgs.msg.Quaternion
             * @static
             * @param {Object.<string,*>} object Plain object
             * @returns {geometry_msgs.msg.Quaternion} Quaternion
             */
            Quaternion.fromObject = function fromObject(object) {
                if (object instanceof $root.geometry_msgs.msg.Quaternion)
                    return object;
                var message = new $root.geometry_msgs.msg.Quaternion();
                if (object.x != null)
                    message.x = Number(object.x);
                if (object.y != null)
                    message.y = Number(object.y);
                if (object.z != null)
                    message.z = Number(object.z);
                if (object.w != null)
                    message.w = Number(object.w);
                return message;
            };

            /**
             * Creates a plain object from a Quaternion message. Also converts values to other types if specified.
             * @function toObject
             * @memberof geometry_msgs.msg.Quaternion
             * @static
             * @param {geometry_msgs.msg.Quaternion} message Quaternion
             * @param {$protobuf.IConversionOptions} [options] Conversion options
             * @returns {Object.<string,*>} Plain object
             */
            Quaternion.toObject = function toObject(message, options) {
                if (!options)
                    options = {};
                var object = {};
                if (options.defaults) {
                    object.x = 0;
                    object.y = 0;
                    object.z = 0;
                    object.w = 0;
                }
                if (message.x != null && message.hasOwnProperty("x"))
                    object.x = options.json && !isFinite(message.x) ? String(message.x) : message.x;
                if (message.y != null && message.hasOwnProperty("y"))
                    object.y = options.json && !isFinite(message.y) ? String(message.y) : message.y;
                if (message.z != null && message.hasOwnProperty("z"))
                    object.z = options.json && !isFinite(message.z) ? String(message.z) : message.z;
                if (message.w != null && message.hasOwnProperty("w"))
                    object.w = options.json && !isFinite(message.w) ? String(message.w) : message.w;
                return object;
            };

            /**
             * Converts this Quaternion to JSON.
             * @function toJSON
             * @memberof geometry_msgs.msg.Quaternion
             * @instance
             * @returns {Object.<string,*>} JSON object
             */
            Quaternion.prototype.toJSON = function toJSON() {
                return this.constructor.toObject(this, $protobuf.util.toJSONOptions);
            };

            /**
             * Gets the default type url for Quaternion
             * @function getTypeUrl
             * @memberof geometry_msgs.msg.Quaternion
             * @static
             * @param {string} [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns {string} The default type url
             */
            Quaternion.getTypeUrl = function getTypeUrl(typeUrlPrefix) {
                if (typeUrlPrefix === undefined) {
                    typeUrlPrefix = "type.googleapis.com";
                }
                return typeUrlPrefix + "/geometry_msgs.msg.Quaternion";
            };

            return Quaternion;
        })();

        return msg;
    })();

    return geometry_msgs;
})();

module.exports = $root;
