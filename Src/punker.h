#pragma once


#define unpack_u8( ptr ) ( ((uint8_t*)(ptr))[0] )
#define unpack_u16( ptr ) (\
                            ( ((uint8_t*)(ptr))[1] << 8 ) |\
                            ( ((uint8_t*)(ptr))[0] ) )
#define unpack_u32( ptr ) (\
                            ( ((uint8_t*)ptr)[3] << 24 ) |\
                            ( ((uint8_t*)ptr)[2] << 16 ) |\
                            ( ((uint8_t*)ptr)[1] << 8 ) |\
                            ( ((uint8_t*)ptr)[0] ) )

#define unpack_float( ptr ) *(float*)(uint32_t[]){ unpack_u32( ptr ) }

#define pack_u8( ptr, data ) ( ((uint8_t*)(ptr))[0] = ( data ) )
#define pack_u16( ptr, data ) do{ \
                                    ((uint8_t*)(ptr))[0] = (data)&0xFF;\
                                    ((uint8_t*)(ptr))[1] = ((data)>>8)&0xFF;\
                                }while(0)
                                  
#define pack_u24( ptr, data ) do{ \
    (uint8_t*)(ptr)[0] = (data)& 0xFF; \
    (uint8_t*)(ptr)[1] = ( ( data ) >> 8 ) & 0xFF; \
    (uint8_t*)(ptr)[2] = ( ( data ) >> 16 ) & 0xFF; \
                                }while( 0 )
                                  
#define pack_u32( _xptr, _xdata ) do{ \
                                    ((uint8_t*)(_xptr))[0] = (_xdata)&0xFF;\
                                    ((uint8_t*)(_xptr))[1] = ((_xdata)>>8)&0xFF;\
                                    ((uint8_t*)(_xptr))[2] = ((_xdata)>>16)&0xFF;\
                                    ((uint8_t*)(_xptr))[3] = ((_xdata)>>24)&0xFF;\
                                }while(0)
                                  
