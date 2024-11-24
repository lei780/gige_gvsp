
#ifndef GEV_GVSP_H
#define GEV__GVSP_H

#include <cstdint>

/**
 * ArvGvspPacketType:
 * @ARV_GVSP_PACKET_TYPE_OK: valid packet
 * @ARV_GVSP_PACKET_TYPE_RESEND: resent packet (BlackFly PointGrey camera support)
 * @ARV_GVSP_PACKET_TYPE_PACKET_UNAVAILABLE: error packet, indicating invalid resend request
 */

typedef enum {
    ARV_GVSP_PACKET_TYPE_OK =       0x0000,
    ARV_GVSP_PACKET_TYPE_RESEND =   0x0100,
    ARV_GVSP_PACKET_TYPE_PACKET_UNAVAILABLE =   0x800c
} ArvGvspPacketType;


/**
 * ArvGvspContentType:
 * @ARV_GVSP_CONTENT_TYPE_LEADER: leader packet
 * @ARV_GVSP_CONTENT_TYPE_TRAILER: trailer packet
 * @ARV_GVSP_CONTENT_TYPE_PAYLOAD: data packet
 * @ARV_GVSP_CONTENT_TYPE_ALL_IN: leader + data + trailer packet
 * @ARV_GVSP_CONTENT_TYPE_H264: h264 data packet
 * @ARV_GVSP_CONTENT_TYPE_MULTIZONE: multizone data packet
 * @ARV_GVSP_CONTENT_TYPE_MULTIPART: multipart data packet
 * @ARV_GVSP_CONTENT_TYPE_GENDC: GenDC data packet
 */

typedef enum {
    ARV_GVSP_CONTENT_TYPE_LEADER =      0x01,
    ARV_GVSP_CONTENT_TYPE_TRAILER =     0x02,
    ARV_GVSP_CONTENT_TYPE_PAYLOAD =     0x03,
    ARV_GVSP_CONTENT_TYPE_ALL_IN =      0x04,
    ARV_GVSP_CONTENT_TYPE_H264 =        0x05,
    ARV_GVSP_CONTENT_TYPE_MULTIZONE =   0x06,
    ARV_GVSP_CONTENT_TYPE_MULTIPART =   0x07,
    ARV_GVSP_CONTENT_TYPE_GENDC =       0x08
} ArvGvspContentType;


/*
 * ArvGvspHeader:
 * @frame_id: frame identifier
 * @packet_infos: #ArvGvspContentType and packet identifier in a 32 bit value
 * @data: data byte array
 *
 * GVSP packet header structure.
 */

typedef struct {
    uint16_t frame_id;
    uint32_t packet_infos;
    uint8_t data[];
} __attribute__((packed)) ArvGvspHeader;


typedef struct {
    uint16_t flags;
    uint32_t packet_infos;
    uint64_t frame_id;
    uint32_t packet_id;
    uint8_t data[];
} __attribute__((packed)) ArvGvspExtendedHeader;

/**
 * ArvGvspLeader:
 * @flags: generic flags
 * @payload_type: ID of the payload type
 */

typedef struct {
    uint16_t flags;
    uint16_t payload_type;
    uint32_t timestamp_high;
    uint32_t timestamp_low;
} __attribute__((packed)) ArvGvspLeader;


typedef struct {
    uint32_t pixel_format;
    uint32_t width;
    uint32_t height;
    uint32_t x_offset;
    uint32_t y_offset;
    uint16_t x_padding;
    uint16_t y_padding;
} ArvGvspImageInfos;

/**
 * ArvGvspImageLeader:
 * @flags: generic flags
 * @payload_type: ID of the payload type
 * @timestamp_high: most significant bits of frame timestamp
 * @timestamp_low: least significant bits of frame timestamp_low
 * @infos: image infos
 */

typedef struct {
    uint16_t flags;
    uint16_t payload_type;
    uint32_t timestamp_high;
    uint32_t timestamp_low;
    ArvGvspImageInfos infos;
} ArvGvspImageLeader;


/**
 * ArvGvspPacket:
 * @packet_type: packet type, also known as status in wireshark dissector
 * @header: common GVSP packet header
 *
 * GVSP packet structure.
 */

typedef struct {
    uint16_t packet_type;
    uint8_t header[];
} __attribute__((packed)) GvspPacket;


#if 0
/* Minimum ethernet frame size minus ethernet protocol overhead */
#define ARV_GVSP_MINIMUM_PACKET_SIZE           (64 - 14 - 4)

/* Maximum ethernet frame size minus ethernet protocol overhead */
#define ARV_GVSP_MAXIMUM_PACKET_SIZE           (65536 - 14 - 4)

 /* IP + UDP */
#define ARV_GVSP_PACKET_UDP_OVERHEAD           (20 + 8)
 /* IP + UDP + GVSP headers or IP + UDP + GVSP extended headers */
#define ARV_GVSP_PACKET_PROTOCOL_OVERHEAD(ext_ids)          ((ext_ids) ? \
                                                                 20 + 8 + \
                                                                 sizeof (ArvGvspPacket) + \
                                                                 sizeof (ArvGvspExtendedHeader) : \
                                                                 20 + 8 + \
                                                                 sizeof (ArvGvspPacket) + \
                                                                 sizeof (ArvGvspHeader))

#define ARV_GVSP_PAYLOAD_PACKET_PROTOCOL_OVERHEAD(ext_ids)      ARV_GVSP_PACKET_PROTOCOL_OVERHEAD(ext_ids)

#define ARV_GVSP_MULTIPART_PACKET_PROTOCOL_OVERHEAD(ext_ids)    ((ext_ids) ? \
                                                                 20 + 8 + \
                                                                 sizeof (ArvGvspPacket) + \
                                                                 sizeof (ArvGvspExtendedHeader) + \
                                                                 sizeof (ArvGvspMultipart) : \
                                                                 20 + 8 + \
                                                                 sizeof (ArvGvspPacket) + \
                                                                 sizeof (ArvGvspHeader) + \
                                                                 sizeof (ArvGvspMultipart))
#endif





#endif /* GEV__GVSP_H */
