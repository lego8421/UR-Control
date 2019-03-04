#include "ur10.hpp"

template <class T>
void change_endian(T *in) {
    unsigned char *memp = reinterpret_cast<unsigned char *>(in);
    for (size_t i = 0; i < sizeof(T) / 2; ++i) {
        std::swap(memp[i], memp[sizeof(T) - i - 1]);
    }
}

void UR10::setBuffer(std::vector<char> buffer) {
    this->buffer = buffer;
    this->packet = convert(this->buffer);
}

Ur10RealTimePacket UR10::getPacket() {
    return packet;
}

Ur10RealTimePacket UR10::convert(std::vector<char> buffer) {
    Ur10RealTimePacket packet;
    memset(&packet, 0, sizeof (packet));

    if (buffer.size() == UR10::PACKET_SIZE) {

        memcpy(&packet.m_dMsgSize, buffer.data(), 4);
        memcpy(&packet.m_fTime, &buffer.data()[4], 8);

        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrQTarget[i], &buffer.data()[12 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrQDTarget[i], &buffer.data()[60 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrQDDTarget[i], &buffer.data()[108 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrITarget[i], &buffer.data()[156 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrMTarget[i], &buffer.data()[204 + (i * 8)], 8);

        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrQActual[i], &buffer.data()[252 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrQDActual[i], &buffer.data()[300 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrIActual[i], &buffer.data()[348 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrIControl[i], &buffer.data()[394 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrToolVectorActual[i], &buffer.data()[444 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrTCPSpeedActual[i], &buffer.data()[492 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrTCPForce[i], &buffer.data()[540 + (i * 8)], 8);

        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrToolVectorTarget[i], &buffer.data()[588 + (i * 8)], 8);
        for (int i = 0; i < 6; i++) memcpy(&packet.m_fArrTCPSpeedTarget[i], &buffer.data()[636 + (i * 8)], 8);
        for (int i = 0; i < 3; i++) memcpy(&packet.m_fArrToolAccelerometerValues[i], &buffer.data()[868 + (i * 8)], 8);

        memcpy(&packet.m_fRobotMode, &buffer.data()[756], 8);
        memcpy(&packet.m_fSafetyMode, &buffer.data()[812], 8);
        memcpy(&packet.m_fProgramState, &buffer.data()[1052], 8);

        change_endian(&(packet.m_dMsgSize));
        change_endian(&(packet.m_fTime));

        for (int i = 0; i < 6; i++) {
            change_endian(&(packet.m_fArrQTarget[i]));        // target joint pos
            change_endian(&(packet.m_fArrQDTarget[i]));       // target joint vel
            change_endian(&(packet.m_fArrQDDTarget[i]));      // target joint acc
            change_endian(&(packet.m_fArrITarget[i]));        // target joint cur
            change_endian(&(packet.m_fArrMTarget[i]));        // target joint cur

            change_endian(&(packet.m_fArrQActual[i]));    // Actual joint pos
            change_endian(&(packet.m_fArrQDActual[i]));   // Actual joint vel
            change_endian(&(packet.m_fArrIActual[i]));    // Actual joint cur

            change_endian(&(packet.m_fArrIControl[i]));    // Joint control currents

            change_endian(&(packet.m_fArrToolVectorActual[i]));   // Actual TCP cartesian pos
            change_endian(&(packet.m_fArrTCPSpeedActual[i]));   // Actual TCP cartesian vel
            change_endian(&(packet.m_fArrTCPForce[i]));   // Actual TCP force

            change_endian(&(packet.m_fArrToolVectorTarget[i]));    // Target Cartesian coordinates of the tool
            change_endian(&(packet.m_fArrTCPSpeedTarget[i]));    // Target speed of the tool given in Cartesian coordinates

        }
        for (int i = 0; i < 3; i++) change_endian(&(packet.m_fArrToolAccelerometerValues[i]));    // Actual  Tool X, y, z accelerometer values

        change_endian(&(packet.m_fRobotMode));
        change_endian(&(packet.m_fSafetyMode));
        change_endian(&(packet.m_fProgramState));
    }
    return packet;
}
