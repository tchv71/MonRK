#pragma once

template <int size>
class CircBuffer
{
public:
    CircBuffer() : m_Ptr(m_Buf), m_End(m_Buf) {}
    bool isEmpty() const { return m_Ptr == m_End; }
    uint8_t getByte()
    {
        uint8_t c = *m_Ptr++;
        if (m_Ptr == m_Buf + sizeof(m_Buf))
            m_Ptr = m_Buf;
        return c;
    }
    void clear()
    {
        m_Ptr = m_End = m_Buf;
    }
    void put(uint8_t c)
    {
        *m_End++ = c;
        if (m_End == m_Buf + sizeof(m_Buf))
            m_End = m_Buf;
    }
    size_t getMaxSize() const { return sizeof(m_Buf); }
    size_t getSize() const
    {
        ptrdiff_t diff = m_End - m_Ptr;
        if (diff >= 0)
            return diff;
        return sizeof(m_Buf) + diff;
    }
    ptrdiff_t remains(int len) { return (m_Buf + sizeof(m_Buf)) - (m_End + len); }
    uint8_t *getPtr() { return m_Ptr; }
    uint8_t *getEndPtr() { return m_End; }
    uint8_t *getStartPtr() { return m_Buf; }
    void movePtr(uint8_t *&Ptr, size_t s)
    {
        Ptr += s;
        const uint8_t *BufEnd = m_Buf + sizeof(m_Buf);
        if (Ptr >= BufEnd)
            Ptr = m_Buf + (Ptr - BufEnd);
    }
    void reserve(size_t s) { movePtr(m_End, s); }
    void advance(size_t s) { movePtr(m_Ptr, s); }
    void setEndToStart(size_t s) { m_End = m_Buf + s; }
    void setCurToEnd() { m_Ptr = m_End; }
    size_t curToEnd() const { return m_Buf + sizeof(m_Buf) - m_Ptr; }
    size_t endToStart() const { return m_End - m_Buf; }

protected:
    uint8_t *m_Ptr;
    uint8_t *m_End;
    uint8_t m_Buf[size];
};
