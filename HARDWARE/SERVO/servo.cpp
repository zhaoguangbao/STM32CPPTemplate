#include "servo.h" 

uint8_t servoErrorCode = 0;

// control table
#define RETURN_DELAY        0x05
#define BLINK_CONDITIONS    0x11
#define SHUTDOWN_CONDITIONS 0x12
#define TORQUE              0x22
#define MAX_SPEED           0x20
#define CURRENT_SPEED       0x26
#define GOAL_ANGLE          0x1e
#define CURRENT_ANGLE       0x24

// global
volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
	
volatile uint8_t* volatile receiveBufferStart = receiveBuffer;
volatile uint8_t* volatile receiveBufferEnd = receiveBuffer;

void clearServoReceiveBuffer (void)
{
    receiveBufferStart = receiveBufferEnd;
}

extern "C" void USART2_IRQHandler (void)
{
	if (USART_GetITStatus (USART2, USART_IT_RXNE))
	{
		const uint8_t byte = (uint8_t)USART_ReceiveData (USART2); // grab the byte from the data register
        
        receiveBufferEnd++;
        if (receiveBufferEnd >= receiveBuffer + REC_BUFFER_LEN)
            receiveBufferEnd = receiveBuffer;
        
        *receiveBufferEnd = byte;
	}
}



// public
// ping a servo, returns true if we get back the expected values
bool Servo::pingServo (const uint8_t servoId)
{
    sendServoCommand (servoId, PING, 0, 0);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool Servo::setServoReturnDelayMicros (const uint8_t servoId,
                                const uint16_t micros)
{
    if (micros > 510)
        return false;
    
    const uint8_t params[2] = {RETURN_DELAY,
                               (uint8_t)((micros / 2) & 0xff)};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to blink its LED
bool Servo::setServoBlinkConditions (const uint8_t servoId,
                              const uint8_t flags)
{
    const uint8_t params[2] = {BLINK_CONDITIONS,
                               flags};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to shut off torque
bool Servo::setServoShutdownConditions (const uint8_t servoId,
                                 const uint8_t flags)
{
    const uint8_t params[2] = {SHUTDOWN_CONDITIONS,
                               flags};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}


// valid torque values are from 0 (free running) to 1023 (max)
bool Servo::setServoTorque (const uint8_t servoId,
                     const uint16_t torqueValue)
{
    const uint8_t highByte = (uint8_t)((torqueValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(torqueValue & 0xff);
    
    if (torqueValue > 1023)
        return false;
    
    const uint8_t params[3] = {TORQUE,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool Servo::getServoTorque (const uint8_t servoId,
                     uint16_t *torqueValue)
{
    const uint8_t params[2] = {TORQUE,
                               2};  // read two bytes, starting at address TORQUE
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *torqueValue = m_response.params[1];
    *torqueValue <<= 8;
    *torqueValue |= m_response.params[0];
    
    return true;
}

// speed values go from 1 (incredibly slow) to 1023 (114 RPM)
// a value of zero will disable velocity control
bool Servo::setServoMaxSpeed (const uint8_t servoId,
                       const uint16_t speedValue)
{
    const uint8_t highByte = (uint8_t)((speedValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(speedValue & 0xff);
    
    if (speedValue > 1023)
        return false;
    
    const uint8_t params[3] = {MAX_SPEED,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool Servo::getServoMaxSpeed (const uint8_t servoId,
                       uint16_t *speedValue)
{
    const uint8_t params[2] = {MAX_SPEED,
                               2};  // read two bytes, starting at address MAX_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *speedValue = m_response.params[1];
    *speedValue <<= 8;
    *speedValue |= m_response.params[0];
    
    return true;
}

bool Servo::getServoCurrentVelocity (const uint8_t servoId,
                              int16_t *velocityValue)
{
    const uint8_t params[2] = {CURRENT_SPEED,
                               2};  // read two bytes, starting at address CURRENT_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *velocityValue = m_response.params[1];
    *velocityValue <<= 8;
    *velocityValue |= m_response.params[0];
    
    return true;
}

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool Servo::setServoAngle (const uint8_t servoId,
                    const float angle)
{
    if (angle < 0 || angle > 300)
        return false;
    
    // angle values go from 0 to 0x3ff (1023)
    const uint16_t angleValue = (uint16_t)(angle * (1023.0 / 300.0));
    
    const uint8_t highByte = (uint8_t)((angleValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angleValue & 0xff);
    
    const uint8_t params[3] = {GOAL_ANGLE,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool Servo::getServoAngle (const uint8_t servoId,
                    float *angle)
{
    const uint8_t params[2] = {CURRENT_ANGLE,
                               2};  // read two bytes, starting at address CURRENT_ANGLE
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    uint16_t angleValue = m_response.params[1];
    angleValue <<= 8;
    angleValue |= m_response.params[0];
    
    *angle = (float)angleValue * 300.0 / 1023.0;
    
    return true;
}



// private
void Servo::sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t numParams,
                       const uint8_t *params)
{
    sendServoByte (0xff);
    sendServoByte (0xff);  // command header
    
    sendServoByte (servoId);  // servo ID
    uint8_t checksum = servoId;
    
    sendServoByte (numParams + 2);  // number of following bytes
    sendServoByte ((uint8_t)commandByte);  // command
    
    checksum += numParams + 2 + commandByte;
    
    for (uint8_t i = 0; i < numParams; i++)
    {
        sendServoByte (params[i]);  // parameters
        checksum += params[i];
    }
    
    sendServoByte (~checksum);  // checksum
}

bool Servo::getServoResponse (void)
{
    uint8_t retries = 0;
    
    clearServoReceiveBuffer();
    
    while (getServoBytesAvailable() < 4)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            user_main_debug("Too many retries at start");
            return false;
        }
        
        delay_us (REC_WAIT_START_US); // delay_ms
    }
    retries = 0;
    
    getServoByte();  // servo header (two 0xff bytes)
    getServoByte();
    
    m_response.id = getServoByte();
    m_response.length = getServoByte();
    
    if (m_response.length > SERVO_MAX_PARAMS)
    {
        user_main_debug("Response length too big: %d", (int)m_response.length);
        return false;
    }
    
    while (getServoBytesAvailable() < m_response.length)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            user_main_debug("Too many retries waiting for params, got %d of %d params", getServoBytesAvailable(), m_response.length);
            return false;
        }
        
        delay_us (REC_WAIT_PARAMS_US);
    }
    
    m_response.error = getServoByte();
    servoErrorCode = m_response.error;
    
    for (uint8_t i = 0; i < m_response.length - 2; i++)
        m_response.params[i] = getServoByte();
    
    
    uint8_t calcChecksum = m_response.id + m_response.length + m_response.error;
    for (uint8_t i = 0; i < m_response.length - 2; i++)
        calcChecksum += m_response.params[i];
    calcChecksum = ~calcChecksum;
    
    const uint8_t recChecksum = getServoByte();
    if (calcChecksum != recChecksum)
    {
        user_main_debug("Checksum mismatch: %x calculated, %x received", calcChecksum, recChecksum);
        return false;
    }
    
    return true;
}

bool Servo::getAndCheckResponse (const uint8_t servoId)
{
    if (!getServoResponse())
    {
        user_main_debug("Servo error: Servo %d did not respond correctly or at all", (int)servoId);
        return false;
    }
    
    if (m_response.id != servoId)
    {
        user_main_debug("Servo error: Response ID %d does not match command ID %d", (int)m_response.id, servoId);
        return false;
    }
    
    if (m_response.error != 0)
    {
        user_main_debug("Servo error: Response error code was nonzero (%d)", (int)m_response.error);
        return false;
    }
    
    return true;
}



// private -- private

int Servo::getServoBytesAvailable (void)
{
    volatile uint8_t *start = receiveBufferStart;
    volatile uint8_t *end = receiveBufferEnd;
    
    if (end >= start)
        return (int)(end - start);
    else
        return (int)(REC_BUFFER_LEN - (start - end));
}

uint8_t Servo::getServoByte (void)
{
    receiveBufferStart++;
    if (receiveBufferStart >= receiveBuffer + REC_BUFFER_LEN)
        receiveBufferStart = receiveBuffer;
    
    return *receiveBufferStart;
}

// private

void Servo::sendServoByte (const uint8_t byte)
{
	  USART_SendData (USART2, (uint16_t)byte);
		USARTx_printf(USART1, "%x ", byte); // test 
	  //Loop until the end of transmission
	  while (USART_GetFlagStatus (USART2, USART_FLAG_TC) == RESET);
}
// usart2
void Servo::initServoUSART()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	clearServoReceiveBuffer();
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
 	USART_DeInit(USART2);  //复位串口1
	 //USART2_TX   PA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //PA2
   
    //USART2_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //PA3


   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = m_baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART2, &USART_InitStructure); //初始化串口
   //Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断

    // set USART2 to half-duplex
    USART_HalfDuplexCmd (USART2, ENABLE);
    
    USART_Cmd(USART2, ENABLE);                    //使能串口 
}
