//////////////////TinyOS随机数组建接口

interface Random
{
    async command uint32_t rand32();//32位伪随机数
    async command uint16_t rand16();//16位伪随机数
}
components CC2530RandomC as RandomC

module CC2530RandomC
{
    provides
    {
        interface Init;
        interface Random;
    }
}

/////////////////////////////////////////////////////////////////////





////Flash组建接口
interface HalFlash
{
    command error_t read(uint8_t *buf,uint32_t address,uint16_t length);
    command error_t write(uint8_t *buf,uint32_t address,uint16_t length);
    command error_t erase(uint32_t address);
}

module HalFlashP
{
    provides interface HalFlash;
    uses interface Dma;
}
//////////////////////////////////////////////////////////////////////////





///////////////DMA组建接口///////////////////
interface Dma
{
    command DMADesc_t *getConfig();//获取DMA配置命令函数
    command error_t armChannal();//装载DMA通道命令函数
    command error_t dosarmChannel();卸载DMA通道命令函数
    command bool isArmd();//判断DMA通道是否装载完成
    command error_t startTransfer();//启动DMA传输
    command error_t stopTransfer();//停止DAM传输
    async event void transferDone();//DMA传输完成事件
}

module DmaP
{
    provides
    {
        interface Init;
        interface Dma[uint8_t id];
    }
}
//////////////////////////////////////////////////////////////

//////////WDT看门组件接口////////////////////////////////
interface WatchDog
{
    command void enable(uint8_t time);//使能看门狗，time为看门狗的时间间隔，取值为0~3
    command void disable();//屏蔽看门狗
    command void clr();//喂看门狗
}

module WatchDogP
{
    provides interface WatchDog;
}

////////////////////////////////////////////////////////////////////

/////////定时器Timer接口定义
interface Timer<precision_tag>
{
    command void starPeriodic(uint32_t);
    command void startOneShot(uint32_t);
    command void stop();
    event void fired();
    command bool isRunning();
    command bool isOneShot();
    command void starPeriodicAt(uint32_t t0,uint32_t dt);
    command void startOneShotAt(uint32_t t0,uint32_t dt);
    command uint32_t getNow()
    command uint32_t gett0();
    command uint32_t getdt();
}

configuration TimerMillip
{
    provides interface Timer<TMilli> as TimerMilli[uint8_t id];
}
//////////////////////////////////////////////////////

///////////ADC接口定义
interface AdcControl
{
    command void enable(uint8_t Ref,uint8_t resolution, uint8_t input);
    command void disable();
}

module AdcP
{
    provides
    {
        interface Init;
        interface AdcControl[uint8_t id];
        interface Read<int16_t>[uint8_t];
    }
}
////////////////////////////////////////////

//////////////////串口通信接口
interface UartByte
{
    async command error_t send(uint8_t byte);
    async command error_t receive(uint8_t *byte,uint8_t timeout);
}

interface UartStream
{
    async command error_t send(uint8_t *buf,uint16_t len);
    async event void sendDone(uint8_t *buf,uint16_t len,error_t error);
    async command error_t enableReceiveInterrupt();
    async command error_t disableReceiveInterrupt();
    async event void receivedByte(uint8_t byte);
    async command error_t receive(uint8_t *buf,uint16_t len);
    async command error receiveDone(uint8_t *buf,uint16_t len,error_t error)
}

interface CC2530UartControl
{
    async command error_t InitUart(uint32_t baud);
    async command error_t setTxInterrupt(uint8_t bIntr);
    async command error_t setRxInterrupt(uint8_t bIntr);
}

module HplCC2530Uart0P
{
    provides
    {
        interface UartStream;
        interface CC2530UartControl;
        interface UartByte;
    }
}

/////////////第八章
////////////////message_t消息结构体
typedef nx_struct message_t
{
    nx_uint8_t header[sizeof(message_header_t)];
    nx_uint8_t data[TOSH_DATA_LENGTH];
    nx_uint8_t footer[sizeof(message_footer_t)];
    nx_uint8_t metadata[sizeof(message_metadata_t)];
}message_t;
//消息头1结构体
typedef union message_header_t
{
    CC2530_header_t cc2530;//消息头
}message_header_t;

 typedef nx_struct CC2530_header_t
 {
     nxle_uint8_t length;//消息头长度
     nxle_uint16_t fcf;//帧控制字段
     nxle_uint8_t dsn;//消息数据序列号
     nxle_uint16_t destpan;//消息目的PAN
     nxle_uint16_t dest;//消息目的地址
     nxle_uint16_t src;//消息源地址
     nxle_uint8_t type;//消息类型
     nx_am_group_t group;//消息组
 }CC2530_header_t;
/////////////2消息元结构体
typedef union message_metadata
{
    CC2530_metadata_t cc2530;
}message_metadata_t;
typedef nx_struct CC2530_metadata_t
{
    nx_uint8_t tx_power;//发送功率
    nx_uint8_t rssi;//接收信号强度指示
    nx_uint8_t lqi;//链路质量指示
    nx_bool crc;//CRC校验
    nx_bool ack;//应答
    nx_uint16_t time;//时间邮戳
}CC2530_metadata_t;

//////////3消息尾结构体定义
typedef union message_footer_t
{
    CC2530_footer_t CC530;//消息尾
}message_metadata_t;


typedef nx_struct CC2530_forget_t
{
    nxle_uint8_t i;
}CC2530_footer_t;


/////////////////基本通信接口
interface Packet
{
    command void clear(message_t* msg);
    command uint8_t payloadLength(message_t* msg);
    command void setPayloadLength(message_t* msg,uint8_t len);
    command uint8_t maxPayloadLength();
    command void getPayload(message_t* msg,uint8_t len);
}

interface Send
{
    command error_t send(message_t* msg,uint8_t len);
    command error_t cancel(message_t* msg);
    event void sendDone(message_t* msg,error_t error);
    command uint8_t maxPayloadLength();
    command void* getPayload(message_t* msg, uint8_t len);
}

interface Receive
{
    event message_t* receive(message_t msg,void* payload, uint8_t len);
}

////////////////主动通信接口AM实现无线通信的多渠道访问机制
interface AMPacket
{
    command am_addr_t address();
    command am_addr_t destination(message_t* amsg);
    command am_addr_t source(message_t* amsg);
    command void setDestination(message_t* amsg,am_addr_t addr);
    command void setSource(message_t* amsg,am_addr_t addr);
    command bool isForMe(message_t* amsg);
    command am_id_t type(message_t* amsg);
    command void command setType(message_t* amsg,am_id_t);
    command am_group_t group (message_t* amsg);
    command void setGroup(message_t* amsg,am_group_t grp);
    command am_group_t LocalGroup();
}

interface AMSend
{
    command error_t send(am_addr_t addr,message_t* msg,uint8_t len);
    command error_t cancel(message_t* msg,error_t error);
    event void sendDone(message_t* msg, error_t error);
    command uint8_t maxPayloadLength();
    command void* getPayload(message_t* msg,uint8_t len);
}

configuration ActiveMessageC
{
    provides
    {
        interface SplitControl;
        interface AMSend[uint8_t];
        interface Receive[uint8_t];
        interface Receive as Snoop[uint8_t id];
        interface Packet;
        interface AMPacket;
        interface PacketAcknowledgements;
    }
}
implementation
{
    components CC2530ActiveMessageC as AM;
    SplitControl=AM;
    AMSend=AM;
    Receive=AM.Receive;
    Snoop=AM.Snoop;
    Packet=AM;
    AMPacket=AM;
}

///////////////CC2530接口与模块
interface CC2530Packet
{
    async command uint8_t getPower(message_t* p_msg);
    async command int8_t getRssi(message_t* p_msg);
    async command void setRssi(message_t* p_msg,uint8_t rssi);
    async command uint8_t getLqi(message_t* p_msg);
}

module CC2530PacketC
{
    provides
    {
        interface CC2530Packet;
        interface PacketAcknowledgements as Acks;
    }
}
///////////CC2530RFControl
interface CC2530RFControl
{
    command error_t sendPacket(uint8_t* packet);
    async event void sendPacketDone(uint8_t* packet,error_t result);
    event uint8_t* receivedPacket(uint8_t* packet);
    command error_t setChannal(uint8_t* packet);
    command uint8_t getChannel();
    command error_t setTransmitPower(uint8_t power);
    command uint8_t getTransmitPower();
    command error_t setAddress(mac_add_t* addr);
    command const mac_addr_t* getAddress();
    command ieee_mac_addr* getExtAddress();
    command error_t setExtAddress(ieee_mac_addr* extAddress);
    command void RxEnable();
    command void RxDisable();
    command error_t setPanAddress(mac_addr_t *addr);
    command const mac_addr_t* getPanAddress();
 }




 ////////CC2530RFInterrupt
 interface CC2530RFInterrupt
 {
    async command error_t enableRFIRQM0(uint8_t IntBit);
    async command error_t disableRFIRQM0(uint8_t IntBit);
    async command error_t enableRFIRQM1(uint8_t IntBit);
    async command error_t disableRFIRQM1(uint8_t IntBit);
    async command error_t enableInterruptRF();
    async command error_t disableInterruptRF();
    async command error_t enableInterruptRFErr();
    async command error_t disableInterruptRFErr();
    async event void RF_TXDONE;
    async event void RF_RXPKTDONE;
 }


 ///////////////////