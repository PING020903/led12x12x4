#include "User_string.h"

/**
 * @brief 根据给定的值合并到目标字节的特定位置
 *
 * 该函数提供了一种根据源值(src)将位(bit)设置或重置到目标字节(dest)的简便方法。
 * 当源值为0时，该函数将目标字节的特定位重置(即，与非操作)；
 * 当源值为非0时，该函数将目标字节的特定位设置(即，或操作)。
 *
 * @param dest 目标字节，将被修改
 * @param src 源值，决定是设置还是重置位
 * @param bit 需要操作的位的位置
 * @return unsigned char 返回修改后的目标字节值
 */
unsigned char merge_value(unsigned char dest,
                          const unsigned char src,
                          const unsigned int bit)
{
    switch (src)
    {
    case 0: // 当源值为0时，重置目标字节的特定位
        return dest &= ~(1 << bit);
    default: // 当源值为非0时，设置目标字节的特定位
        return dest |= (1 << bit);
    }

    return dest;
}