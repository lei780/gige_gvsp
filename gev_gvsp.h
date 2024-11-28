
#ifndef GEV_GVSP_H
#define GEV__GVSP_H

#include <cstdio>
#include <cstdint>


#define PVC_CAMERA_MEMORY_SIZE 0x10000

#define PVC_GVSP_PACKET_EXTENDED_ID_MODE_MASK   0x80
#define PVC_GVSP_PACKET_ID_MASK                 0x00ffffff
#define PVC_GVSP_PACKET_INFOS_CONTENT_TYPE_MASK 0x7f000000
#define PVC_GVSP_PACKET_INFOS_CONTENT_TYPE_POS  24
#define PVC_GVSP_PACKET_INFOS_N_PARTS_MASK      0x000000ff

#define MAXBUF 1536


/**
 * ArvGvspPacketType:
 * @ARV_GVSP_PACKET_TYPE_OK: valid packet
 * @ARV_GVSP_PACKET_TYPE_RESEND: resent packet (BlackFly PointGrey camera support)
 * @ARV_GVSP_PACKET_TYPE_PACKET_UNAVAILABLE: error packet, indicating invalid resend request
 */

typedef enum {
    PVC_GVSP_PACKET_TYPE_OK =       0x0000,
    PVC_GVSP_PACKET_TYPE_RESEND =   0x0100,
    PVC_GVSP_PACKET_TYPE_PACKET_UNAVAILABLE =   0x800c
} PvcGvspPacketType;

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
    PVC_GVSP_CONTENT_TYPE_LEADER =      0x01,
    PVC_GVSP_CONTENT_TYPE_TRAILER =     0x02,
    PVC_GVSP_CONTENT_TYPE_PAYLOAD =     0x03,
    PVC_GVSP_CONTENT_TYPE_ALL_IN =      0x04,
    PVC_GVSP_CONTENT_TYPE_H264 =        0x05,
    PVC_GVSP_CONTENT_TYPE_MULTIZONE =   0x06,
    PVC_GVSP_CONTENT_TYPE_MULTIPART =   0x07,
    PVC_GVSP_CONTENT_TYPE_GENDC =       0x08
} PvcGvspContentType;


typedef enum {
    PVC_GVSP_CONTENT_TYPE_LEADER =      0x01,
    PVC_GVSP_CONTENT_TYPE_TRAILER =     0x02,
    PVC_GVSP_CONTENT_TYPE_PAYLOAD =     0x03,
    PVC_GVSP_CONTENT_TYPE_ALL_IN =      0x04,
    PVC_GVSP_CONTENT_TYPE_H264 =        0x05,
    PVC_GVSP_CONTENT_TYPE_MULTIZONE =   0x06,
    PVC_GVSP_CONTENT_TYPE_MULTIPART =   0x07,
    PVC_GVSP_CONTENT_TYPE_GENDC =       0x08
} PvcGvspContentType;


typedef enum {
    PVC_BUFFER_PAYLOAD_TYPE_UNKNOWN =           -1,
    PVC_BUFFER_PAYLOAD_TYPE_NO_DATA =       0x0000,
    PVC_BUFFER_PAYLOAD_TYPE_IMAGE =         0x0001,
    PVC_BUFFER_PAYLOAD_TYPE_RAWDATA =       0x0002,
    PVC_BUFFER_PAYLOAD_TYPE_FILE =          0x0003,
    PVC_BUFFER_PAYLOAD_TYPE_CHUNK_DATA =    0x0004,
    PVC_BUFFER_PAYLOAD_TYPE_EXTENDED_CHUNK_DATA = 0x0005, /* Deprecated */
    PVC_BUFFER_PAYLOAD_TYPE_JPEG =          0x0006,
    PVC_BUFFER_PAYLOAD_TYPE_JPEG2000 =      0x0007,
    PVC_BUFFER_PAYLOAD_TYPE_H264 =          0x0008,
    PVC_BUFFER_PAYLOAD_TYPE_MULTIZONE_IMAGE = 0x0009,
    PVC_BUFFER_PAYLOAD_TYPE_MULTIPART =             0x000a,
    PVC_BUFFER_PAYLOAD_TYPE_GENDC_CONTAINER =       0x000b,
    PVC_BUFFER_PAYLOAD_TYPE_GENDC_COMPONENT_DATA =  0x000c
} PvcBufferPayloadType;



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
} __attribute__((packed)) PvcGvspHeader;


typedef struct {
    uint16_t flags;
    uint32_t packet_infos;
    uint64_t frame_id;
    uint32_t packet_id;
    uint8_t data[];
} __attribute__((packed)) PvcGvspExtendedHeader;

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
} __attribute__((packed)) PvcGvspLeader;


typedef struct {
    uint32_t pixel_format;
    uint32_t width;
    uint32_t height;
    uint32_t x_offset;
    uint32_t y_offset;
    uint16_t x_padding;
    uint16_t y_padding;
} __attribute__((packed)) PvcGvspImageInfos;


typedef struct {
    uint16_t flags;
    uint16_t payload_type;
    uint32_t timestamp_high;
    uint32_t timestamp_low;
    PvcGvspImageInfos infos;
} __attribute__((packed))  PvcGvspImageLeader;


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
