import * as $protobuf from "protobufjs";
/** Namespace std_msgs. */
export namespace std_msgs {

    /** Namespace msg. */
    namespace msg {

        /** Properties of a Header. */
        interface IHeader {

            /** Header stamp */
            stamp?: (std_msgs.msg.ITime|null);

            /** Header frame_id */
            frame_id?: (string|null);
        }

        /** Represents a Header. */
        class Header implements IHeader {

            /**
             * Constructs a new Header.
             * @param [properties] Properties to set
             */
            constructor(properties?: std_msgs.msg.IHeader);

            /** Header stamp. */
            public stamp?: (std_msgs.msg.ITime|null);

            /** Header frame_id. */
            public frame_id: string;

            /**
             * Creates a new Header instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Header instance
             */
            public static create(properties?: std_msgs.msg.IHeader): std_msgs.msg.Header;

            /**
             * Encodes the specified Header message. Does not implicitly {@link std_msgs.msg.Header.verify|verify} messages.
             * @param message Header message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: std_msgs.msg.IHeader, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Header message, length delimited. Does not implicitly {@link std_msgs.msg.Header.verify|verify} messages.
             * @param message Header message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: std_msgs.msg.IHeader, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Header message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Header
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): std_msgs.msg.Header;

            /**
             * Decodes a Header message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Header
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): std_msgs.msg.Header;

            /**
             * Verifies a Header message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Header message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Header
             */
            public static fromObject(object: { [k: string]: any }): std_msgs.msg.Header;

            /**
             * Creates a plain object from a Header message. Also converts values to other types if specified.
             * @param message Header
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: std_msgs.msg.Header, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Header to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Header
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Time. */
        interface ITime {

            /** Time sec */
            sec?: (number|null);

            /** Time nanosec */
            nanosec?: (number|null);
        }

        /** Represents a Time. */
        class Time implements ITime {

            /**
             * Constructs a new Time.
             * @param [properties] Properties to set
             */
            constructor(properties?: std_msgs.msg.ITime);

            /** Time sec. */
            public sec: number;

            /** Time nanosec. */
            public nanosec: number;

            /**
             * Creates a new Time instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Time instance
             */
            public static create(properties?: std_msgs.msg.ITime): std_msgs.msg.Time;

            /**
             * Encodes the specified Time message. Does not implicitly {@link std_msgs.msg.Time.verify|verify} messages.
             * @param message Time message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: std_msgs.msg.ITime, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Time message, length delimited. Does not implicitly {@link std_msgs.msg.Time.verify|verify} messages.
             * @param message Time message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: std_msgs.msg.ITime, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Time message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Time
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): std_msgs.msg.Time;

            /**
             * Decodes a Time message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Time
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): std_msgs.msg.Time;

            /**
             * Verifies a Time message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Time message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Time
             */
            public static fromObject(object: { [k: string]: any }): std_msgs.msg.Time;

            /**
             * Creates a plain object from a Time message. Also converts values to other types if specified.
             * @param message Time
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: std_msgs.msg.Time, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Time to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Time
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a UInt32. */
        interface IUInt32 {

            /** UInt32 data */
            data?: (number|null);
        }

        /** Represents a UInt32. */
        class UInt32 implements IUInt32 {

            /**
             * Constructs a new UInt32.
             * @param [properties] Properties to set
             */
            constructor(properties?: std_msgs.msg.IUInt32);

            /** UInt32 data. */
            public data: number;

            /**
             * Creates a new UInt32 instance using the specified properties.
             * @param [properties] Properties to set
             * @returns UInt32 instance
             */
            public static create(properties?: std_msgs.msg.IUInt32): std_msgs.msg.UInt32;

            /**
             * Encodes the specified UInt32 message. Does not implicitly {@link std_msgs.msg.UInt32.verify|verify} messages.
             * @param message UInt32 message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: std_msgs.msg.IUInt32, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified UInt32 message, length delimited. Does not implicitly {@link std_msgs.msg.UInt32.verify|verify} messages.
             * @param message UInt32 message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: std_msgs.msg.IUInt32, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a UInt32 message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns UInt32
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): std_msgs.msg.UInt32;

            /**
             * Decodes a UInt32 message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns UInt32
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): std_msgs.msg.UInt32;

            /**
             * Verifies a UInt32 message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a UInt32 message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns UInt32
             */
            public static fromObject(object: { [k: string]: any }): std_msgs.msg.UInt32;

            /**
             * Creates a plain object from a UInt32 message. Also converts values to other types if specified.
             * @param message UInt32
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: std_msgs.msg.UInt32, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this UInt32 to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for UInt32
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a String. */
        interface IString {

            /** String data */
            data?: (string|null);
        }

        /** Represents a String. */
        class String implements IString {

            /**
             * Constructs a new String.
             * @param [properties] Properties to set
             */
            constructor(properties?: std_msgs.msg.IString);

            /** String data. */
            public data: string;

            /**
             * Creates a new String instance using the specified properties.
             * @param [properties] Properties to set
             * @returns String instance
             */
            public static create(properties?: std_msgs.msg.IString): std_msgs.msg.String;

            /**
             * Encodes the specified String message. Does not implicitly {@link std_msgs.msg.String.verify|verify} messages.
             * @param message String message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: std_msgs.msg.IString, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified String message, length delimited. Does not implicitly {@link std_msgs.msg.String.verify|verify} messages.
             * @param message String message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: std_msgs.msg.IString, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a String message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns String
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): std_msgs.msg.String;

            /**
             * Decodes a String message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns String
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): std_msgs.msg.String;

            /**
             * Verifies a String message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a String message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns String
             */
            public static fromObject(object: { [k: string]: any }): std_msgs.msg.String;

            /**
             * Creates a plain object from a String message. Also converts values to other types if specified.
             * @param message String
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: std_msgs.msg.String, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this String to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for String
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }
}

/** Namespace geometry_msgs. */
export namespace geometry_msgs {

    /** Namespace msg. */
    namespace msg {

        /** Properties of a PoseStamped. */
        interface IPoseStamped {

            /** PoseStamped header */
            header?: (std_msgs.msg.IHeader|null);

            /** PoseStamped pose */
            pose?: (geometry_msgs.msg.IPose|null);
        }

        /** Represents a PoseStamped. */
        class PoseStamped implements IPoseStamped {

            /**
             * Constructs a new PoseStamped.
             * @param [properties] Properties to set
             */
            constructor(properties?: geometry_msgs.msg.IPoseStamped);

            /** PoseStamped header. */
            public header?: (std_msgs.msg.IHeader|null);

            /** PoseStamped pose. */
            public pose?: (geometry_msgs.msg.IPose|null);

            /**
             * Creates a new PoseStamped instance using the specified properties.
             * @param [properties] Properties to set
             * @returns PoseStamped instance
             */
            public static create(properties?: geometry_msgs.msg.IPoseStamped): geometry_msgs.msg.PoseStamped;

            /**
             * Encodes the specified PoseStamped message. Does not implicitly {@link geometry_msgs.msg.PoseStamped.verify|verify} messages.
             * @param message PoseStamped message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: geometry_msgs.msg.IPoseStamped, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified PoseStamped message, length delimited. Does not implicitly {@link geometry_msgs.msg.PoseStamped.verify|verify} messages.
             * @param message PoseStamped message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: geometry_msgs.msg.IPoseStamped, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a PoseStamped message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns PoseStamped
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): geometry_msgs.msg.PoseStamped;

            /**
             * Decodes a PoseStamped message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns PoseStamped
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): geometry_msgs.msg.PoseStamped;

            /**
             * Verifies a PoseStamped message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a PoseStamped message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns PoseStamped
             */
            public static fromObject(object: { [k: string]: any }): geometry_msgs.msg.PoseStamped;

            /**
             * Creates a plain object from a PoseStamped message. Also converts values to other types if specified.
             * @param message PoseStamped
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: geometry_msgs.msg.PoseStamped, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this PoseStamped to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for PoseStamped
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Pose. */
        interface IPose {

            /** Pose position */
            position?: (geometry_msgs.msg.IPoint|null);

            /** Pose orientation */
            orientation?: (geometry_msgs.msg.IQuaternion|null);
        }

        /** Represents a Pose. */
        class Pose implements IPose {

            /**
             * Constructs a new Pose.
             * @param [properties] Properties to set
             */
            constructor(properties?: geometry_msgs.msg.IPose);

            /** Pose position. */
            public position?: (geometry_msgs.msg.IPoint|null);

            /** Pose orientation. */
            public orientation?: (geometry_msgs.msg.IQuaternion|null);

            /**
             * Creates a new Pose instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Pose instance
             */
            public static create(properties?: geometry_msgs.msg.IPose): geometry_msgs.msg.Pose;

            /**
             * Encodes the specified Pose message. Does not implicitly {@link geometry_msgs.msg.Pose.verify|verify} messages.
             * @param message Pose message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: geometry_msgs.msg.IPose, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Pose message, length delimited. Does not implicitly {@link geometry_msgs.msg.Pose.verify|verify} messages.
             * @param message Pose message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: geometry_msgs.msg.IPose, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Pose message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Pose
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): geometry_msgs.msg.Pose;

            /**
             * Decodes a Pose message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Pose
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): geometry_msgs.msg.Pose;

            /**
             * Verifies a Pose message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Pose message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Pose
             */
            public static fromObject(object: { [k: string]: any }): geometry_msgs.msg.Pose;

            /**
             * Creates a plain object from a Pose message. Also converts values to other types if specified.
             * @param message Pose
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: geometry_msgs.msg.Pose, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Pose to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Pose
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Point. */
        interface IPoint {

            /** Point x */
            x?: (number|null);

            /** Point y */
            y?: (number|null);

            /** Point z */
            z?: (number|null);
        }

        /** Represents a Point. */
        class Point implements IPoint {

            /**
             * Constructs a new Point.
             * @param [properties] Properties to set
             */
            constructor(properties?: geometry_msgs.msg.IPoint);

            /** Point x. */
            public x: number;

            /** Point y. */
            public y: number;

            /** Point z. */
            public z: number;

            /**
             * Creates a new Point instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Point instance
             */
            public static create(properties?: geometry_msgs.msg.IPoint): geometry_msgs.msg.Point;

            /**
             * Encodes the specified Point message. Does not implicitly {@link geometry_msgs.msg.Point.verify|verify} messages.
             * @param message Point message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: geometry_msgs.msg.IPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Point message, length delimited. Does not implicitly {@link geometry_msgs.msg.Point.verify|verify} messages.
             * @param message Point message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: geometry_msgs.msg.IPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Point message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): geometry_msgs.msg.Point;

            /**
             * Decodes a Point message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): geometry_msgs.msg.Point;

            /**
             * Verifies a Point message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Point message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Point
             */
            public static fromObject(object: { [k: string]: any }): geometry_msgs.msg.Point;

            /**
             * Creates a plain object from a Point message. Also converts values to other types if specified.
             * @param message Point
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: geometry_msgs.msg.Point, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Point to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Point
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Quaternion. */
        interface IQuaternion {

            /** Quaternion x */
            x?: (number|null);

            /** Quaternion y */
            y?: (number|null);

            /** Quaternion z */
            z?: (number|null);

            /** Quaternion w */
            w?: (number|null);
        }

        /** Represents a Quaternion. */
        class Quaternion implements IQuaternion {

            /**
             * Constructs a new Quaternion.
             * @param [properties] Properties to set
             */
            constructor(properties?: geometry_msgs.msg.IQuaternion);

            /** Quaternion x. */
            public x: number;

            /** Quaternion y. */
            public y: number;

            /** Quaternion z. */
            public z: number;

            /** Quaternion w. */
            public w: number;

            /**
             * Creates a new Quaternion instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Quaternion instance
             */
            public static create(properties?: geometry_msgs.msg.IQuaternion): geometry_msgs.msg.Quaternion;

            /**
             * Encodes the specified Quaternion message. Does not implicitly {@link geometry_msgs.msg.Quaternion.verify|verify} messages.
             * @param message Quaternion message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: geometry_msgs.msg.IQuaternion, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Quaternion message, length delimited. Does not implicitly {@link geometry_msgs.msg.Quaternion.verify|verify} messages.
             * @param message Quaternion message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: geometry_msgs.msg.IQuaternion, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Quaternion message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Quaternion
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): geometry_msgs.msg.Quaternion;

            /**
             * Decodes a Quaternion message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Quaternion
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): geometry_msgs.msg.Quaternion;

            /**
             * Verifies a Quaternion message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Quaternion message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Quaternion
             */
            public static fromObject(object: { [k: string]: any }): geometry_msgs.msg.Quaternion;

            /**
             * Creates a plain object from a Quaternion message. Also converts values to other types if specified.
             * @param message Quaternion
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: geometry_msgs.msg.Quaternion, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Quaternion to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Quaternion
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }
}
