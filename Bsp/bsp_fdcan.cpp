#include "stm32h7xx_hal.h"
#include "bsp_fdcan.h"
#include "main.h"
#include"Bsp_DJI_Motor.h"
#include "app_entry.h"

struct fdcan_motor_feedback GM6020_feedback[4]{};

volatile uint32_t fdcan_rx_count;
volatile uint32_t fdcan_rx_id = 0U;
volatile uint32_t fdcan_rx_dlc = 0U;
volatile uint8_t fdcan_rx_data[8]{};

TxFrame frames[3] =
{
    {0x1FF,{}},
    {0x200,{}},
    {0x2FF,{}}
};

//FDCAN初始化函数
void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
  //打开FDCAN外设
  HAL_FDCAN_Start(hfdcan);
  //激活FDCAN中断
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
}

//FDCAN滤波函数
void FDCAN_FILTER_MASK_Config(FDCAN_HandleTypeDef *hfdcan,uint32_t id,uint32_t mask_id)
{
  /* 1. 定义过滤器配置结构体。
   *    HAL 约定的“规则表格”，必须用 {0} 初始化：
   *    把结构体所有成员先清 0，避免没填的字段带着栈上的垃圾值。
   *    局部变量没问题：HAL 在调用期间把值读走、写进硬件，
   *    函数返回后这个变量就可以销毁了。 */
  FDCAN_FilterTypeDef fdcan_filter_init_structure = {0};

  /* 2. IdType：这次配置的是“哪一张过滤器表”。
   *    FDCAN 内部有两张独立的表，各有各的 RAM 空间：
   *      标准帧表（11 位 ID） → FDCAN_STANDARD_ID = 0x00000000
   *      扩展帧表（29 位 ID） → FDCAN_EXTENDED_ID = 0x40000000
   *    你的 CubeMX 配置是 StdFiltersNbr = 1、ExtFiltersNbr = 0，
   *    只给标准帧表分配了空间，所以只能选 FDCAN_STANDARD_ID。
   *    对应 F4 教程里的“标准帧/扩展帧”（bxCAN 靠寄存器里的 IDE 位表达）。 */
  fdcan_filter_init_structure.IdType = FDCAN_STANDARD_ID;

  /* 3. FilterIndex：这张表里的“第几号过滤器元素”（编号，从 0 开始）。
   *    你的 StdFiltersNbr = 1，只有 1 个槽位，所以只能填 0；
   *    填 1 会触发 HAL 的 assert_param 检查失败。
   *    注意：它不是“过滤器数量”（数量在 CubeMX 配），而是“用第几个槽”。
   *    对应 F4 教程的 FilterBank。 */
  fdcan_filter_init_structure.FilterIndex = 0;

  /* 4. FilterType：ID 和掩码“怎么比较”。
   *      FDCAN_FILTER_MASK  = 0x00000002 → 经典掩码模式：
   *                          FilterID1 是要匹配的 ID，FilterID2 是掩码
   *      FDCAN_FILTER_RANGE = 0x00000000 → 范围模式：
   *                          FilterID1 是下限，FilterID2 是上限
   *      FDCAN_FILTER_DUAL  = 0x00000001 → 双 ID 模式：
   *                          FilterID1、FilterID2 是两个独立 ID，任一匹配即收
   *    对应 F4 教程的 FilterMode = CAN_FILTERMODE_IDMASK。 */
  fdcan_filter_init_structure.FilterType = FDCAN_FILTER_MASK;

  /* 5. FilterConfig：匹配上的帧“存到哪里”。
   *      FDCAN_FILTER_TO_RXFIFO0    = 0x00000001 → 放进接收 FIFO0
   *      FDCAN_FILTER_TO_RXFIFO1    = 0x00000002 → 放进接收 FIFO1
   *      FDCAN_FILTER_TO_RXFIFO0_HP = 0x00000005 → 高优先级，进 FIFO0
   *      FDCAN_FILTER_TO_RXFIFO1_HP = 0x00000006 → 高优先级，进 FIFO1
   *    你的工程 RxFifo0ElmtsNbr = 16、RxFifo1ElmtsNbr = 0，
   *    只给 FIFO0 分配了 16 个格子，所以只能选 FIFO0。
   *    它决定了后面用哪套中断和读取函数：
   *      中断：FDCAN_IT_RX_FIFO0_NEW_MESSAGE
   *      读取：HAL_FDCAN_GetRxMessage(..., FDCAN_RX_FIFO0, ...)
   *    对应 F4 教程的 Fil       terFIFOAssignment。 */
  fdcan_filter_init_structure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

  /* 6. FilterID1：要匹配的 ID（掩码模式下的“模板”）。
   *    标准帧 11 位，合法范围 0 ~ 0x7FF。
   *    直接写 ID 真值，不要像 F4 那样 <<5：
   *    FDCAN 的 HAL 内部会自己做 (FilterID1 << 16) 拼进 Message RAM 元素字。
   *    （& 0x7FF 只是兜底，防止传入超出 11 位的值。）
   *    例：GM6020 反馈 ID = 0x204 + 拨码 ID。 */
  fdcan_filter_init_structure.FilterID1 = id & 0x7FF;

  /* 7. FilterID2：掩码（只有在 FilterType = FDCAN_FILTER_MASK 时才是掩码）。
   *    规则：某一位 = 1 → ID 的对应位必须和 FilterID1 相同
   *          某一位 = 0 → 这一位不关心，0/1 都能匹配
   *    常用取值：
   *      0x7FF → 11 位全比较 → 精确匹配
   *      0x7F0 → 高 7 位比较、低 4 位随意 → 收 0x200~0x20F
   *      0x000 → 一位都不比较 → 全收（调试用）
   *    它不是“二号 ID”——那是 FDCAN_FILTER_DUAL 模式下的含义。
   *    对应 F4 教程的 FilterMaskIdHigh / FilterMaskIdLow。 */
  fdcan_filter_init_structure.FilterID2 = mask_id & 0x7FF;

  /* 8. 两种模式下用不到的字段（不用填，{0} 已清零，HAL 会忽略）：
   *      RxBufferIndex    → 只有 FilterConfig = FDCAN_FILTER_TO_RXBUFFER
   *                         （进专用接收缓冲）时才用，指定用哪个缓冲
   *      IsCalibrationMsg → 同样只在专用缓冲模式用，标记“校准报文”
   *    你的工程 RxBuffersNbr = 0，根本没有专用接收缓冲，所以这两个字段无意义。 */

  /* 9. 写入硬件：HAL 把上面几个值拼成 Message RAM 里的过滤器元素字。
   *    失败（如槽位越界）返回 HAL_ERROR。
   *    注意 FDCAN 没有 F4 的 FilterActivation 使能位——写进去就生效；
   *    也没有 FilterScale——标准帧元素天生 32 位、扩展帧元素 64 位。 */
  if (HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter_init_structure) != HAL_OK)
  {
    Error_Handler();   /* 停在死循环，方便调试器立刻发现配置失败 */
  }

  /* 10. 全局过滤（GFC 寄存器）：没匹配任何过滤器、或远端帧怎么处理。
   *    前两个参数：不匹配的标准帧 / 不匹配的扩展帧
   *      FDCAN_REJECT             = 0x00000002 → 拒收（正常做法）
   *      FDCAN_ACCEPT_IN_RX_FIFO0 = 0x00000000 → 收进 FIFO0
   *      FDCAN_ACCEPT_IN_RX_FIFO1 = 0x00000001 → 收进 FIFO1
   *    后两个参数：标准/扩展“远端帧（Remote Frame）”怎么处理
   *      FDCAN_REJECT_REMOTE = 0x00000001 → 拒收
   *      FDCAN_FILTER_REMOTE = 0x00000000 → 按过滤器规则处理
   *    你的应用只收数据帧，所以四个都填拒收。
   *    这一步在 F4 的 bxCAN 里没有对应操作：bxCAN 不匹配就直接丢掉。 */
  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
}

//发送报文函数
uint8_t FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan,uint32_t id,uint8_t *data,uint8_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader = {0};   /* 9 个字段先全部清 0 */

  if ((hfdcan == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }
  if (len > 8U)                           /* 经典 CAN 一帧最多 8 字节 */
  {
    return HAL_ERROR;
  }

  TxHeader.Identifier          = id & 0x7FFU;   //仲裁段：11位标准帧ID，范围0~0x7FF
  TxHeader.TxFrameType         = FDCAN_DATA_FRAME;//仲裁段RTR(remote transmission request)：0表示数据帧，1表示遥控帧
  TxHeader.IdType              = FDCAN_STANDARD_ID;//控制段IDE(identifier extension)：0表示标准帧，1表示扩展帧
  TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;//控制段r0：0表示经典CAN，1表示FD CAN
  TxHeader.DataLength          = len;           //控制段DLC(data length code)：数据长度码，0~8表示0~8字节，9~15表示12~64字节（从9开始每增加1表示增加4字节）
  //数据段：Data
  //校验段：CRC15 -- 硬件计算
  //校验段：CRC Delimiter = 1 --硬件生成
  //应答段：ACK slot = 1 -- 发送端发 1，收到有效帧的接收节点可用 0 应答 -- CAN 硬件处理
  //应答段：ACK Delimiter = 1，标记应答段结束 -- 硬件生成
  //结束段	EOF · End of Frame	7	1111111，标记一帧结束	硬件生成
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;//CAN FD 帧里的错误状态指示位，经典帧里没有 ESI
  TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;//Bit Rate Switch，CAN FD 是否切换数据阶段速率，经典帧全程使用同一个速率。
  TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;//Event FIFO Control，是否把发送事件记入本机的 Tx Event FIFO，FDCAN_NO_TX_EVENTS，关闭本机发送事件记录。
  TxHeader.MessageMarker       = 0U;//写入发送事件的 8 位软件标记，开启事件记录时，可以用这个标记认出记录对应哪一帧。

  return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
}



void BSP_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  const uint32_t pending = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
  for (uint32_t n = 0U; n < pending; ++n)
  {
    FDCAN_RxHeaderTypeDef RxHeader{};
    uint8_t RxData[64]{};  /* HAL 按 DLC 复制数据，缓冲区容纳其最大长度。 */

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      break;
    }

    APP_OnFDCAN_RX(RxHeader,RxData);

    fdcan_rx_id = RxHeader.Identifier;
    fdcan_rx_dlc = RxHeader.DataLength;
    for (uint32_t i = 0U; i < sizeof(fdcan_rx_data); ++i)
    {
      fdcan_rx_data[i] = RxData[i];
    }
    ++fdcan_rx_count;
  }
}



