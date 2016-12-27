/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AT_C4_2040_CHUNKMODEDEFS
#define AT_C4_2040_CHUNKMODEDEFS

#include "stdint.h"
#include <QDebug>

#pragma pack(push)
#pragma pack(1)

typedef struct _GV_ChunkAcqInfo
{
    unsigned int timeStamp64L; // 0..3
    unsigned int timeStamp64H; // 4..7
    unsigned int frameCnt; // 8..11
    signed int triggerCoord; // 12..15
    unsigned char triggerStatus; // 16
    unsigned short DAC; // 17..18
    unsigned short ADC; // 19..20
    unsigned char INT_idx; // 21
    unsigned char AOI_idx; // 22
    unsigned short AOI_ys; // 23..24
    unsigned short AOI_dy; // 25..26
    unsigned short AOI_xs; // 27..28
    unsigned short AOI_trsh; // 29..30
    unsigned char AOI_alg; // 31
} GV_ChunkAcqInfo;

#define CHUNKACQINFO_TRIGGERSTATUS_BIT_TRIGGER_OVERRUN 0x01
#define CHUNKACQINFO_TRIGGERSTATUS_BIT_RESOLVER_CNT_UP 0x02
#define CHUNKACQINFO_TRIGGERSTATUS_BIT_IN0 0x10
#define CHUNKACQINFO_TRIGGERSTATUS_BIT_IN1 0x20
#define CHUNKACQINFO_TRIGGERSTATUS_BIT_OUT0 0x40
#define CHUNKACQINFO_TRIGGERSTATUS_BIT_OUT1 0x80

typedef struct _GV_ChunkImageInfo
{
    unsigned int mSizeYReal;
    unsigned int numChunkAcqInfo;
    unsigned int flag;
} GV_ChunkImageInfo;

#define CHUNKIMAGEINFO_FLAG_BIT_START_FRAME 0x00000001
#define CHUNKIMAGEINFO_FLAG_BIT_STOP_FRAME 0x00000002

typedef struct _GV_ChunkDescriptor
{
    unsigned int descriptor;
    unsigned int length;
} GV_ChunkDescriptorData;

#pragma pack(pop)



namespace AT {
namespace C4_2040 {

void processChunkData(uint8_t *data, uint32_t dataSize) {
    const size_t chunkDescriptorSize = sizeof GV_ChunkDescriptorData;

    const uint8_t *dataEnd = data + dataSize;
    uint32_t processedDataBytes = 0;
    GV_ChunkDescriptorData lastChunkDescriptor;

    int numChunkAcqInfo = -1;
    while (processedDataBytes < dataSize) {
        const uint8_t *lastChunkDescriptorBegin =
                dataEnd - processedDataBytes - chunkDescriptorSize;
        processedDataBytes += 8;

    //                        for (int i=0; i<chunkDescriptorSize; ++i)
    //                            qDebug() << lastChunkDescriptorBegin[i];

        /// GV_ChunkDescriptorData from the data array is in big endian order
        lastChunkDescriptor.descriptor = (((uint32_t) lastChunkDescriptorBegin[0]) << 3*8)
                                       + (((uint32_t) lastChunkDescriptorBegin[1]) << 2*8)
                                       + (((uint32_t) lastChunkDescriptorBegin[2]) << 1*8)
                                       + (((uint32_t) lastChunkDescriptorBegin[3]));
        lastChunkDescriptor.length = (((uint32_t) lastChunkDescriptorBegin[4]) << 3*8)
                                   + (((uint32_t) lastChunkDescriptorBegin[5]) << 2*8)
                                   + (((uint32_t) lastChunkDescriptorBegin[6]) << 1*8)
                                   + (((uint32_t) lastChunkDescriptorBegin[7]));

        fprintf(stdout, "descriptor: 0x%.8X length %d\n", lastChunkDescriptor.descriptor, lastChunkDescriptor.length);

        switch (lastChunkDescriptor.descriptor) {
        case 0x11119999: { /// ChunkImageInfo
            const GV_ChunkImageInfo *chunkImageInfo = (GV_ChunkImageInfo *)(lastChunkDescriptorBegin - lastChunkDescriptor.length);
            if (numChunkAcqInfo < 0) {
                numChunkAcqInfo = chunkImageInfo->numChunkAcqInfo;
            } else {
                qWarning() << "repeated ChunkImageInfo?";
            }
            qDebug() << "[ChunkImageInfo]"
                     << "flag:" << chunkImageInfo->flag
                     << "mSizeYReal:" << chunkImageInfo->mSizeYReal
                     << "numChunkAcqInfo:" << chunkImageInfo->numChunkAcqInfo;
            break;
        }
        case 0x66669999: { /// ChunkAcqInfo
            if (numChunkAcqInfo < 0) {
                qWarning() << "ChunkImageInfo not processed yet?";
            } else {
                const GV_ChunkAcqInfo *chunkAcqInfo = (GV_ChunkAcqInfo *)(lastChunkDescriptorBegin - lastChunkDescriptor.length);
                for (int cai = 0; cai<numChunkAcqInfo; ++cai, chunkAcqInfo++) {
                    qDebug() << "[ChunkAcqInfo" << cai << "]"
                             << "ADC" << chunkAcqInfo->ADC
                             << "AOI_alg" << chunkAcqInfo->AOI_alg
                             << "AOI_dy" << chunkAcqInfo->AOI_dy
                             << "AOI_idx" << chunkAcqInfo->AOI_idx
                             << "AOI_trsh" << chunkAcqInfo->AOI_trsh
                             << "AOI_xs" << chunkAcqInfo->AOI_xs
                             << "AOI_ys" << chunkAcqInfo->AOI_ys
                             << "DAC" << chunkAcqInfo->DAC
                             << "frameCnt" << chunkAcqInfo->frameCnt
                             << "INT_idx" << chunkAcqInfo->INT_idx
                             << "timeStamp64H" << chunkAcqInfo->timeStamp64H
                             << "timeStamp64L" << chunkAcqInfo->timeStamp64L
                             << "triggerCoord" << chunkAcqInfo->triggerCoord
                             << "triggerStatus" << chunkAcqInfo->triggerStatus;
                }
                //const size_t chunkAcqInfoSize = sizeof GV_ChunkAcqInfo;
            }
            break;
        }
        case 0xA5A5A5A5: { /// ChunkImage
            qDebug() << "[ChunkImage]";
            break;
        }
        default:
            qWarning() << "unknown chunk descriptor";
            break;
        }

        processedDataBytes += lastChunkDescriptor.length;
    }
}

}
}

#endif // AT_C4_2040_CHUNKMODEDEFS
