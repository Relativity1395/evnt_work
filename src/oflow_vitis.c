#include <hls_stream.h>
#include <ap_int.h>


typedef ap_axiu<64,0,0,0> strm_pkt
hls::stream<strm_pkt> in_stream;
hls::stream<strm_pkt> out_stream;

void stream_function(
hls::stream<strm_pkt> &in_stream,
hls::stream<strm_pkt> &out_stream,
){
#pragma HLS INTERFACE axis port = in_stream
#pragma HLS INTERFACE axis port = out_stream
ap_uint<10> x = in_stream.data.range(64,55);
ap_uint<9> y = in_stream.data.range(55,47);
uint32_t t = in_stream.data.range(55,24);
bool p = in_stream.data.range(24,24);

}