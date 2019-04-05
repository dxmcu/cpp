/*
 * SeerProtocol.h
 *
 *  Created on: Jun 7, 2018
 *      Author: fengwz
 */

#ifndef SEERPROTOCOL_H_
#define SEERPROTOCOL_H_

#include <string.h>
#ifndef WIN32
# include <endian.h>
#endif
#include <boost/log/trivial.hpp>

namespace chassis
{

#pragma pack(push, 1)

struct ProtocolHeader {
    uint8_t m_sync;
    uint8_t m_version;
    uint16_t m_number;
    uint32_t m_length;
    uint16_t m_type;
    uint8_t m_reserved[6];

    ProtocolHeader()
    {
        m_sync = 0x5A;
        m_version = 1;
        m_number = 0;
        m_length = 0;
        m_type = 0;
        memset(m_reserved, 0, sizeof (m_reserved));
    }
};

#pragma pack(pop)

class ProtocolMsg
{
public:
    ProtocolHeader m_proHeader;
    std::string m_strJsonBody;

    ProtocolMsg(bool bNetwork = false)
        : m_bNetwork(bNetwork)
    {
    }

    void Dump()
    {
        BOOST_LOG_TRIVIAL(info) << "DUMP: " << (m_bNetwork ? "(network)" : "(host)");

        BOOST_LOG_TRIVIAL(info) << "<ProtocolMsg> " << (int)m_proHeader.m_sync << "-"
                                << (int)m_proHeader.m_version << "-" << (int)m_proHeader.m_number << "-"
                                << (int)m_proHeader.m_length << "-" << (int)m_proHeader.m_type
                                << "-" << m_strJsonBody;

        BOOST_LOG_TRIVIAL(info) << "END";
    }

    void HtoN()
    {
        if(!m_bNetwork)
        {
#ifndef WIN32
            m_proHeader.m_number = htobe16(m_proHeader.m_number);
            m_proHeader.m_length = htobe32(m_proHeader.m_length);
            m_proHeader.m_type = htobe16(m_proHeader.m_type);
#endif
            m_bNetwork = true;
        }
    }

    void NtoH()
    {
        if(m_bNetwork)
        {
#ifndef WIN32
            m_proHeader.m_number = be16toh(m_proHeader.m_number);
            m_proHeader.m_length = be32toh(m_proHeader.m_length);
            m_proHeader.m_type = be16toh(m_proHeader.m_type);
#endif
            m_bNetwork = false;
        }
    }

private:
    bool m_bNetwork;
};

} /* namespace seer */

#endif /* SEERPROTOCOL_H_ */
