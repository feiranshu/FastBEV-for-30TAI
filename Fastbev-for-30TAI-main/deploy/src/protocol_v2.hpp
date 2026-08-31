#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <zlib.h>
namespace advwire {
constexpr uint16_t kVersion=2,kMessagePerceptionBatch=10,kMessagePerceptionResult=11,kMessageControlInput=20,kMessageControlResult=21;
constexpr size_t kHeaderBytes=32,kCameraEntryBytes=24; constexpr uint32_t kMaxPacketBytes=64U*1024U*1024U;
inline uint16_t be16(const uint8_t*p){return (uint16_t(p[0])<<8U)|uint16_t(p[1]);}
inline uint32_t be32(const uint8_t*p){return (uint32_t(p[0])<<24U)|(uint32_t(p[1])<<16U)|(uint32_t(p[2])<<8U)|uint32_t(p[3]);}
inline uint64_t be64(const uint8_t*p){uint64_t v=0;for(int i=0;i<8;++i)v=(v<<8U)|uint64_t(p[i]);return v;}
inline void put16(std::vector<uint8_t>&o,uint16_t v){o.push_back(uint8_t(v>>8));o.push_back(uint8_t(v));}
inline void put32(std::vector<uint8_t>&o,uint32_t v){o.push_back(uint8_t(v>>24));o.push_back(uint8_t(v>>16));o.push_back(uint8_t(v>>8));o.push_back(uint8_t(v));}
inline void put64(std::vector<uint8_t>&o,uint64_t v){for(int s=56;s>=0;s-=8)o.push_back(uint8_t(v>>s));}
struct Header{uint16_t type=0;uint64_t frame_id=0,timestamp_ns=0;uint32_t item_count=0,payload_size=0;};
inline Header parse_header(const std::vector<uint8_t>&p){
 if(p.size()<kHeaderBytes)throw std::runtime_error("ADV2 short header");
 if(std::memcmp(p.data(),"ADV2",4)!=0)throw std::runtime_error("bad ADV2 magic");
 if(be16(p.data()+4)!=kVersion)throw std::runtime_error("bad ADV2 version");
 Header h;h.type=be16(p.data()+6);h.frame_id=be64(p.data()+8);h.timestamp_ns=be64(p.data()+16);h.item_count=be32(p.data()+24);h.payload_size=be32(p.data()+28);
 if(h.payload_size!=p.size()-kHeaderBytes)throw std::runtime_error("ADV2 payload mismatch");return h;
}
struct PerceptionBatch{uint64_t frame_id=0,capture_ts_ns=0;std::string metadata_json;std::array<std::vector<uint8_t>,6> jpeg;};
template<typename Order> inline PerceptionBatch parse_perception_batch(const std::vector<uint8_t>&p,const Order&o){
 Header h=parse_header(p);if(h.type!=kMessagePerceptionBatch||h.item_count!=6)throw std::runtime_error("expected 6-camera ADV2 batch");
 size_t off=kHeaderBytes;if(off+4>p.size())throw std::runtime_error("missing metadata length");uint32_t mb=be32(p.data()+off);off+=4;if(off+mb>p.size())throw std::runtime_error("truncated metadata");
 PerceptionBatch b;b.frame_id=h.frame_id;b.capture_ts_ns=h.timestamp_ns;b.metadata_json.assign(reinterpret_cast<const char*>(p.data()+off),mb);off+=mb;
 for(size_t i=0;i<6;++i){if(off+kCameraEntryBytes>p.size())throw std::runtime_error("truncated camera entry");size_t n=0;while(n<16&&p[off+n])++n;std::string name(reinterpret_cast<const char*>(p.data()+off),n);uint32_t bytes=be32(p.data()+off+16),crc=be32(p.data()+off+20);off+=kCameraEntryBytes;if(name!=std::string(o[i]))throw std::runtime_error("camera order mismatch "+name);if(bytes<4||off+bytes>p.size())throw std::runtime_error("bad JPEG length");b.jpeg[i].assign(p.begin()+off,p.begin()+off+bytes);off+=bytes;uLong actual=crc32(0L,Z_NULL,0);actual=crc32(actual,reinterpret_cast<const Bytef*>(b.jpeg[i].data()),static_cast<uInt>(b.jpeg[i].size()));if(uint32_t(actual)!=crc)throw std::runtime_error("JPEG CRC mismatch");}
 if(off!=p.size())throw std::runtime_error("trailing batch bytes");return b;
}
struct JsonMessage{Header header;std::string json;};
inline JsonMessage parse_json_message(const std::vector<uint8_t>&p,uint16_t expected){JsonMessage m;m.header=parse_header(p);if(m.header.type!=expected)throw std::runtime_error("wrong ADV2 type");m.json.assign(reinterpret_cast<const char*>(p.data()+kHeaderBytes),p.size()-kHeaderBytes);return m;}
inline std::vector<uint8_t> pack_json(uint16_t type,uint64_t frame,uint64_t ts,uint32_t items,const std::string&j){std::vector<uint8_t>o;o.reserve(kHeaderBytes+j.size());o.insert(o.end(),{'A','D','V','2'});put16(o,kVersion);put16(o,type);put64(o,frame);put64(o,ts);put32(o,items);put32(o,uint32_t(j.size()));o.insert(o.end(),j.begin(),j.end());return o;}
}