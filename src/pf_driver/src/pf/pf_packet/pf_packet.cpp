// #include <algorithm>
#include <iostream>

#include "pf_driver/pf/pf_packet/pf_packet.h"

bool PFPacket::parse_buf(uint8_t* buf, size_t buf_len, size_t& remainder, size_t& packet_size)
{
  const size_t SIZE = get_size();
  // check if buffer is large enough for the header
  if (buf_len < SIZE)
    return false;

  boost::shared_array<uint8_t> buffer(new uint8_t[SIZE]);
  std::copy(buf, buf + SIZE, buffer.get());

  uint16_t h_size;
  uint32_t p_size;
  uint16_t num;
  std::tie(h_size, p_size, num) = read_header(buffer);

  // advertised header size must not be greater than size of message
  // NOTE: The get_size() - function will return the size of the header including padding,
  // so comparison to equality will fail!
  if (h_size > SIZE)
    return false;

  // check if the buffer is large enough to contain the entire packet
  if (buf_len < p_size)
    return false;

  // calculate the size of the point data based on the number of points and the size of each point
  const size_t packet_data_size = get_point_data_size() * num;

  // check if the packet size is sufficient to hold the point data and the header
  if (p_size < packet_data_size + h_size)
    return false;

  read_data(&buf[h_size], num);
  remainder = buf_len - p_size;
  packet_size = p_size;
  return true;
}

int PFPacket::find_packet_start(uint8_t* buf, size_t buf_len)
{
  char p_type[2];
  get_type(p_type);
  for (size_t i = 0; i < buf_len - 4; i++)
  {
    if (((unsigned char)buf[i]) == 0x5c && ((unsigned char)buf[i + 1]) == 0xa2 &&
        ((unsigned char)buf[i + 2]) == p_type[0] && ((unsigned char)buf[i + 3]) == p_type[1])
    {
      return i;
    }
  }
  return -1;
}
